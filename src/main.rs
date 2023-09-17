//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

extern crate panic_halt;

use cortex_m::delay::Delay;
// Alias for our HAL crate
use rp2040_hal as hal;

use hal::{clocks::init_clocks_and_plls, gpio::I2C, pac, Clock};

// Some traits we need
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::{Ref, RefCell};
use critical_section::Mutex;
use rp2040_hal::gpio;
// The macro for our start-up function

// info!() and error!() macros for printing information to the debug output
use defmt::*;
use defmt_rtt as _;

// Embed the `Hz` function/trait:
use fugit::RateExtU32;
mod display;

use itoa;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const DISPLAY_ADRESS: u8 = 0b00001111;
const DECODE_MODE_ADRESS: u8 = 0b00001001;
const INTENSITY_ADRESS: u8 = 0b00001010;
const SCAN_LIMIT_ADRESS: u8 = 0b00001011;
const SHUTDOWN: u8 = 0x0C;

const ADRESS: u8 = 0x27; // 0x27 or 0x3F
use rp2040_hal::gpio::Interrupt::EdgeLow;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)

type DisplayPub = Option<
    display::Display<
        hal::I2C<
            pac::I2C1,
            (
                gpio::Pin<gpio::bank0::Gpio18, gpio::Function<I2C>>,
                gpio::Pin<gpio::bank0::Gpio19, gpio::Function<I2C>>,
            ),
        >,
    >,
>;

//Current buttons: P4-: 26, P4+: 8, P3-: 9 p3+: 10

type P4Minus = gpio::Pin<gpio::bank0::Gpio26, gpio::Input<gpio::PullUp>>;
type P4Plus = gpio::Pin<gpio::bank0::Gpio8, gpio::Input<gpio::PullUp>>;
type P3Minus = gpio::Pin<gpio::bank0::Gpio9, gpio::Input<gpio::PullUp>>;
type P3Plus = gpio::Pin<gpio::bank0::Gpio10, gpio::Input<gpio::PullUp>>;

enum InterruptButtons {
    P3Minus(P3Minus),
    P3Plus(P3Plus),
    P4Minus(P4Minus),
    P4Plus(P4Plus),
}

type Buttons = [InterruptButtons; 4];

type LEDPin = gpio::Pin<gpio::bank0::Gpio25, gpio::Output<gpio::PushPull>>;

type PublicDisplay = display::Display<
    hal::I2C<
        pac::I2C1,
        (
            gpio::Pin<gpio::bank0::Gpio18, gpio::Function<I2C>>,
            gpio::Pin<gpio::bank0::Gpio19, gpio::Function<I2C>>,
        ),
    >,
>;

static GLOBAL_PINS: Mutex<RefCell<Option<Buttons>>> = Mutex::new(RefCell::new(None));
static GLOBAL_DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static GLOBAL_LED_PIN: Mutex<RefCell<Option<LEDPin>>> = Mutex::new(RefCell::new(None));

static GLOBAL_DISPLAY: Mutex<RefCell<Option<PublicDisplay>>> = Mutex::new(RefCell::new(None));

static LIFE_TOTALS: Mutex<RefCell<[i32; 4]>> = Mutex::new(RefCell::new([40, 40, 40, 40]));
#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        pins.gpio18.into_mode(),
        pins.gpio19.into_mode(),
        100000.Hz(),
        &mut pac.RESETS,
        12_000_000.Hz(),
    );

    let mut display: display::Display<
        hal::I2C<
            pac::I2C1,
            (
                gpio::Pin<gpio::bank0::Gpio18, gpio::Function<I2C>>,
                gpio::Pin<gpio::bank0::Gpio19, gpio::Function<I2C>>,
            ),
        >,
    > = display::Display::new(ADRESS, i2c);
    display.init(&mut delay).unwrap();

    let p3Minus = pins.gpio9.into_pull_up_input();
    p3Minus.set_interrupt_enabled(EdgeLow, true);
    let p3Plus = pins.gpio10.into_pull_up_input();
    p3Plus.set_interrupt_enabled(EdgeLow, true);
    let p4Minus = pins.gpio26.into_pull_up_input();
    p4Minus.set_interrupt_enabled(EdgeLow, true);
    let p4Plus = pins.gpio8.into_pull_up_input();
    p4Plus.set_interrupt_enabled(EdgeLow, true);

    let mut led_pin: gpio::Pin<gpio::bank0::Gpio25, gpio::Output<gpio::PushPull>> =
        pins.gpio25.into_push_pull_output();
    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some([
            InterruptButtons::P3Minus(p3Minus),
            InterruptButtons::P3Plus(p3Plus),
            InterruptButtons::P4Minus(p4Minus),
            InterruptButtons::P4Plus(p4Plus),
        ]));
        GLOBAL_DELAY.borrow(cs).replace(Some(delay));
        GLOBAL_LED_PIN.borrow(cs).replace(Some(led_pin));
        GLOBAL_DISPLAY.borrow(cs).replace(Some(display));
    });

    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    let mut counter = 0;
    loop {
        //display.write(0, 0, "THOOOOOOR JOIN", &mut delay).unwrap();
        //display.write(0, 1, "BALDERS PORT!", &mut delay).unwrap();
        critical_section::with(|cs| {
            let optionedDelay = GLOBAL_DELAY.borrow(cs).take();
            let optionedDisplay = GLOBAL_DISPLAY.borrow(cs).take();
            let lifeTotals = LIFE_TOTALS.borrow(cs).take();
            if optionedDelay.is_some() && optionedDisplay.is_some() {
                let mut delay = optionedDelay.unwrap();
                let mut display = optionedDisplay.unwrap();

                let mut buffer = itoa::Buffer::new();
                let mut positions = [(0, 0), (8, 0), (0, 1), (8, 1)];
                for (i, v) in lifeTotals.iter().enumerate() {
                    let (x, y) = positions[i];

                    let _ = display.write(x, y, buffer.format(*v), &mut delay);
                }
                GLOBAL_DELAY.borrow(cs).replace(Some(delay));
                GLOBAL_DISPLAY.borrow(cs).replace(Some(display));
                LIFE_TOTALS.borrow(cs).replace(lifeTotals);
            }
        });
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut Buttons: Option<Buttons> = None;

    if Buttons.is_none() {
        critical_section::with(|cs| {
            *Buttons = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(gpios) = Buttons {
        let buttons = gpios;

        for i in buttons {
            match i {
                InterruptButtons::P3Minus(a) => {
                    if a.interrupt_status(EdgeLow) {
                        grabCriticalSectionAndChangeNum(-1, 3);
                        a.clear_interrupt(EdgeLow)
                    }
                }
                InterruptButtons::P3Plus(a) => {
                    if a.interrupt_status(EdgeLow) {
                        grabCriticalSectionAndChangeNum(1, 3);
                        a.clear_interrupt(EdgeLow)
                    }
                }
                InterruptButtons::P4Minus(a) => {
                    if a.interrupt_status(EdgeLow) {
                        grabCriticalSectionAndChangeNum(-1, 4);
                        a.clear_interrupt(EdgeLow)
                    }
                }
                InterruptButtons::P4Plus(a) => {
                    if a.interrupt_status(EdgeLow) {
                        grabCriticalSectionAndChangeNum(1, 4);
                        a.clear_interrupt(EdgeLow)
                    }
                }
            }
        }
    }
}

fn grabCriticalSectionAndChangeNum(diff: i32, player: u8) {
    critical_section::with(|cs| {
        let mut lifeTotals = LIFE_TOTALS.borrow(cs).take();
        lifeTotals[player as usize - 1] += diff;
        LIFE_TOTALS.borrow(cs).replace(lifeTotals);
    })
}
// End of file
