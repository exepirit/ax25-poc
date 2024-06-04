//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::str::FromStr;

extern crate alloc;
use alloc::vec::Vec;
use ax25::frame::{
    Address, Ax25Frame, CommandResponse, FrameContent, ProtocolIdentifier, UnnumberedInformation,
};
use bsp::{
    entry,
    hal::{
        fugit::HertzU32,
        uart::{self, UartPeripheral},
    },
};
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_alloc::Heap;
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();

    // setup UART on pin 4 (RX) and pin 5 (TX)
    let uart = uart::UartPeripheral::new(
        pac.UART1,
        (pins.gpio4.into_function(), pins.gpio5.into_function()),
        &mut pac.RESETS,
    )
    .enable(
        uart::UartConfig::new(
            HertzU32::Hz(115200),
            uart::DataBits::Eight,
            None,
            uart::StopBits::One,
        ),
        clocks.peripheral_clock.freq(),
    )
    .unwrap();

    loop {
        led_pin.set_high().unwrap();
        send_test_frame(&uart);
        led_pin.set_low().unwrap();

        delay.delay_ms(3000);
    }
}

fn send_test_frame<D: uart::UartDevice, P: uart::ValidUartPinout<D>>(
    uart: &UartPeripheral<uart::Enabled, D, P>,
) {
    let frame = Ax25Frame {
        source: Address::from_str("NOCALL-1").unwrap(),
        destination: Address::from_str("NOCALL-2").unwrap(),
        route: Vec::new(),
        command_or_response: Some(CommandResponse::Command),
        content: FrameContent::UnnumberedInformation(UnnumberedInformation {
            pid: ProtocolIdentifier::None,
            poll_or_final: false,
            info: Vec::from("Hello, World!".as_bytes()),
        }),
    };
    uart.write_full_blocking(&frame.to_bytes());
}

// End of file
