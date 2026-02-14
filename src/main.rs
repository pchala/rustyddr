#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1, USB};
use embassy_rp::gpio::Pull;
use embassy_rp::pac;
use embassy_rp::pio::{
    Config, Direction, FifoJoin, InterruptHandler as PioInterruptHandler, Pio, ShiftDirection, StateMachine,
};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use fixed::traits::ToFixed;
use pio::pio_asm;

use defmt_rtt as _; 
use panic_probe as _; 

// ============================================================================
// INTERRUPTS
// ============================================================================
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>; 
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>; 
});

// ============================================================================
// USB STATIC BUFFERS
// ============================================================================
static mut CONFIG_DESC: [u8; 256] = [0; 256];
static mut BOS_DESC: [u8; 256] = [0; 256];
static mut MSOS_DESC: [u8; 256] = [0; 256];
static mut CONTROL_BUF: [u8; 64] = [0; 64];
static mut HID_STATE: embassy_usb::class::hid::State = embassy_usb::class::hid::State::new();

// ============================================================================
// RAW HID DESCRIPTOR (Gamepad, 16 Buttons, No Axes)
// ============================================================================
const HID_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x05, // USAGE (Gamepad)
    0xa1, 0x01, // COLLECTION (Application)
    
    0x05, 0x09, //   USAGE_PAGE (Button)
    0x19, 0x01, //   USAGE_MINIMUM (Button 1)
    0x29, 0x10, //   USAGE_MAXIMUM (Button 16)
    
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x25, 0x01, //   LOGICAL_MAXIMUM (1)
    0x75, 0x01, //   REPORT_SIZE (1 bit)
    0x95, 0x10, //   REPORT_COUNT (16)
    
    0x81, 0x02, //   INPUT (Data, Var, Abs)
    0xc0,       // END_COLLECTION
];

// ============================================================================
// MAIN APPLICATION
// ============================================================================
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Starting Pro-Sync DDR Firmware...");

    // ------------------------------------------------------------------------
    // 1. SETUP USB HID
    // ------------------------------------------------------------------------
    let driver = Driver::new(p.USB, Irqs);
    let mut config = embassy_usb::Config::new(0x1234, 0x1114);
    config.manufacturer = Some("EmbassyDDR");
    config.product = Some("Pro-Sync-DDR"); 
    config.max_power = 400;

    let mut builder = unsafe {
        embassy_usb::Builder::new(
            driver, config,
            &mut *core::ptr::addr_of_mut!(CONFIG_DESC),
            &mut *core::ptr::addr_of_mut!(BOS_DESC),
            &mut *core::ptr::addr_of_mut!(MSOS_DESC),
            &mut *core::ptr::addr_of_mut!(CONTROL_BUF),
        )
    };

    let hid_config = embassy_usb::class::hid::Config {
        report_descriptor: HID_REPORT_DESCRIPTOR,
        request_handler: None,
        poll_ms: 1, 
        max_packet_size: 64,
    };

    let mut hid_writer = unsafe {
        embassy_usb::class::hid::HidWriter::<_, 2>::new(
            &mut builder, &mut *core::ptr::addr_of_mut!(HID_STATE), hid_config
        )
    };
    
    spawner.spawn(usb_task(builder.build())).unwrap();

    // ------------------------------------------------------------------------
    // 2. SETUP GAMECUBE MATS (PIO0 at 2MHz)
    // ------------------------------------------------------------------------
    let joybus_src = pio_asm!(
        ".side_set 1 opt",
        ".wrap_target",
        
        // --- SETUP TX ---
        "pull block",                    
        "out x, 32",                     
        
        "pull block",                    
        "set pindirs, 1      side 1",    
        
        // --- TRANSMIT PHASE ---
        "out y, 1            side 1",    
        
        "tx_loop:",
        "jmp !y do_zero      side 0",    
        
        "do_one:",
        "nop                 side 0 [1]",    
        "out y, 1            side 1 [2]",
        "jmp x-- tx_loop     side 1 [2]",
        "jmp stop_bit        side 1",    
        
        "do_zero:",
        "nop                 side 0 [4]",
        "out y, 1            side 0",    
        "jmp x-- tx_loop     side 1 [1]",
        
        "stop_bit:",
        "nop                 side 0 [1]",
        "set pindirs, 0      side 1",    
        
        // --- SETUP RX ---
        "pull block",                    
        "out y, 32",                     
        
        // --- RECEIVE PHASE ---
        "rx_loop:",
        "wait 0 pin 0",                  
        "nop                 [2]",       
        "in pins, 1",                    
        "wait 1 pin 0",                  
        "jmp y-- rx_loop",               
        
        "push block",                    
        ".wrap"
    );

    let Pio { common: mut pio0_common, sm0: mut mat1, sm1: mut mat2, .. } = Pio::new(p.PIO0, Irqs);
    
    let mut gc_pin0 = pio0_common.make_pio_pin(p.PIN_0);
    gc_pin0.set_pull(Pull::Up);
    let mut gc_pin1 = pio0_common.make_pio_pin(p.PIN_1);
    gc_pin1.set_pull(Pull::Up);

    let joy_prog = pio0_common.load_program(&joybus_src.program);

    macro_rules! create_gc_config {
        ($pin:expr) => {{
            let mut cfg = Config::default();
            cfg.use_program(&joy_prog, &[$pin]); 
            cfg.set_out_pins(&[$pin]);
            cfg.set_in_pins(&[$pin]);
            cfg.set_set_pins(&[$pin]); 
            cfg.clock_divider = (125_000_000 / 2_000_000).to_fixed(); 
            
            cfg.shift_in.auto_fill = true;
            cfg.shift_in.threshold = 32;
            
            cfg.shift_out.auto_fill = false; 
            cfg.shift_in.direction = ShiftDirection::Left;  
            cfg.shift_out.direction = ShiftDirection::Left; 
            cfg
        }};
    }

    mat1.set_config(&create_gc_config!(&gc_pin0));
    mat1.set_pin_dirs(Direction::In, &[&gc_pin0]); 

    mat2.set_config(&create_gc_config!(&gc_pin1));
    mat2.set_pin_dirs(Direction::In, &[&gc_pin1]);

    // ------------------------------------------------------------------------
    // 3. SETUP LEDS (PIO1, Pin 16)
    // ------------------------------------------------------------------------
    let ws2812_src = pio_asm!(
        ".side_set 1",
        ".wrap_target",
        "get_data:",
        "pull block      side 0",      // STALL: Forces line LOW when FIFO is empty!
        "set y, 23       side 0",      // Loop 24 times for 1 LED
        "bitloop:",
        "out x, 1        side 0 [2]",  
        "jmp !x do_zero  side 1 [1]",  
        "do_one:",
        "jmp y-- bitloop side 1 [4]",  // Long High
        "jmp get_data    side 0",      // Done 24 bits. Force line LOW.
        "do_zero:",
        "jmp y-- bitloop side 0 [4]",  // Long Low
        ".wrap"
    );

    let Pio { common: mut pio1_common, sm0: mut leds, .. } = Pio::new(p.PIO1, Irqs);
    let led_pin16 = pio1_common.make_pio_pin(p.PIN_16);
    let led_prog = pio1_common.load_program(&ws2812_src.program);
    
    let mut led_cfg = Config::default();
    led_cfg.use_program(&led_prog, &[&led_pin16]); 
    led_cfg.set_out_pins(&[&led_pin16]);
    led_cfg.clock_divider = (125_000_000 / 8_000_000).to_fixed(); 
    led_cfg.shift_out.direction = ShiftDirection::Left; 
    led_cfg.shift_out.auto_fill = false; 
    led_cfg.fifo_join = FifoJoin::TxOnly;

    leds.set_config(&led_cfg);
    leds.set_pin_dirs(Direction::Out, &[&led_pin16]);
    leds.set_enable(true);

    // ------------------------------------------------------------------------
    // 4. HANDSHAKE
    // ------------------------------------------------------------------------
    info!("Waking up Dance Mats...");
    pac::PIO0.ctrl().modify(|w| w.set_sm_enable(3)); 

    mat1.tx().push(8 - 1); mat1.tx().push(0x00 << 24); mat1.tx().push(24 - 1);
    mat2.tx().push(8 - 1); mat2.tx().push(0x00 << 24); mat2.tx().push(24 - 1);

    let _ = mat1.rx().wait_pull().await; // Read Mat 1 ID
    let _ = mat2.rx().wait_pull().await; // Read Mat 2 ID

    info!("Handshake complete. Entering Competition Loop!");

    // ------------------------------------------------------------------------
    // 5. MAIN POLLING LOOP
    // ------------------------------------------------------------------------
    loop {
        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(0)); 

        // Poll Command (24 bits TX, 64 bits RX) but we will take only buttons
        mat1.tx().push(24 - 1); mat1.tx().push(0x400300 << 8); mat1.tx().push(16 - 1);
        mat2.tx().push(24 - 1); mat2.tx().push(0x400300 << 8); mat2.tx().push(16 - 1);

        pac::PIO0.ctrl().modify(|w| w.set_sm_enable(3)); 

        // We MUST read all 3 words to perfectly drain the hardware buffer!
        let m1_buttons = mat1.rx().wait_pull().await; // Word 1: Contains D-Pad & Action Buttons
        // let _m1_analog = mat1.rx().wait_pull().await; // Word 2: Contains Joystick data
        // let _m1_flush  = mat1.rx().wait_pull().await; // Word 3: Empty flush from PIO

        let m2_buttons = mat2.rx().wait_pull().await; 
        // let _m2_analog = mat2.rx().wait_pull().await; 
        // let _m2_flush  = mat2.rx().wait_pull().await; 

        let p1_btns = decode_buttons(m1_buttons);
        let p2_btns = decode_buttons(m2_buttons);
        
        let combined = (p1_btns as u16) | ((p2_btns as u16) << 8);

        match hid_writer.write(&combined.to_le_bytes()).await {
            Ok(_) => {},
            Err(_) => {}, 
        }

        update_leds(&mut leds, p1_btns, p2_btns);

        // sleep for 300us to have fresher samples next time
        Timer::after_micros(300).await;

    }
}

// ----------------------------------------------------------------------------
// HELPER FUNCTIONS
// ----------------------------------------------------------------------------

fn decode_buttons(val: u32) -> u8 {
    let b0 = (val >> 8) as u8; 
    let b1 = (val) as u8; 
    
    (b1 & 0x01) |                 // Left  (Bit 0 stays Bit 0)
    ((b1 & 0x0C) >> 1) |          // Down & Up (Bits 2,3 shift to Bits 1,2)
    ((b1 & 0x02) << 2) |          // Right (Bit 1 shifts to Bit 3)
    (b0 & 0x10) |                 // Start (Bit 4 stays Bit 4)
    ((b1 & 0x10) << 1) |          // Z     (Bit 4 shifts to Bit 5)
    ((b0 & 0x03) << 6)            // A & B (Bits 0,1 shift to Bits 6,7)
}

fn update_leds(sm: &mut StateMachine<'_, PIO1, 0>, p1: u8, p2: u8) {
    let c_p1 = 0x10_00_00_00; // Dim Green
    let c_p2 = 0x00_00_10_00; // Dim Blue
    let off = 0;

    for i in 0..4 { 
        sm.tx().push(if (p1 & (1 << i)) != 0 { c_p1 } else { off }); 
    }
    for i in 0..4 { 
        sm.tx().push(if (p2 & (1 << i)) != 0 { c_p2 } else { off }); 
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}