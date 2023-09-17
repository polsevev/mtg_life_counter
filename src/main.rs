//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{
        clocks,
        gpio::{
            bank0::{Gpio18, Gpio19},
            Function, FunctionI2C, FunctionSpi, Pin,
        },
        i2c::I2C,
        pwm::A,
        Spi,
    },
    pac::I2C1,
    Pins,
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    digital::v2::OutputPin,
    prelude::{_embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write},
    spi::{FullDuplex, MODE_0},
};
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

use core::cell::RefCell;

// The macro for our start-up function

// info!() and error!() macros for printing information to the debug output
use defmt::*;
use defmt_rtt as _;

// Embed the `Hz` function/trait:
use fugit::RateExtU32;

mod display;
const DISPLAY_ADRESS: u8 = 0b00001111;
const DECODE_MODE_ADRESS: u8 = 0b00001001;
const INTENSITY_ADRESS: u8 = 0b00001010;
const SCAN_LIMIT_ADRESS: u8 = 0b00001011;
const SHUTDOWN: u8 = 0x0C;

const ADRESS: u8 = 0x27; // 0x27 or 0x3F

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio18.into_mode(),
        pins.gpio19.into_mode(),
        100000.Hz(),
        &mut pac.RESETS,
        12_000_000.Hz(),
    );

    let mut display = display::Display::new(ADRESS, i2c);
    display.init(&mut delay).unwrap();

    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();
    let to_write = [
        "THIS", "IS", "A", "MESSAGE", "FROM", "OUR", "SPONSORS", "RAID", "SHADOW", "LEGENDS!",
    ];

    let mut counter = 0;
    loop {
        display.write(0, 0, "THOOOOOOR JOIN", &mut delay).unwrap();
        display.write(0, 1, "BALDERS PORT!", &mut delay).unwrap();
        counter += 1;
        if counter == to_write.len() {
            counter = 0;
        }
        led_pin.set_high().unwrap();
        delay.delay_ms(1000);
        led_pin.set_low().unwrap();
        delay.delay_ms(1000);
    }
}

// End of file
