//! Receive packets using the LoRa 1276 module on the Adafruit Feather RP2040 RFM95 board.
//!
//! This will trigger the RxDone interrupt on DIO0 if successful and blink the on-board LED. On
//! failure, the on-board LED will be turned on.

#![no_std]
#![no_main]

extern crate sx127x_lora;

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::asm::wfi;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal_bus::spi::RefCellDevice;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::{FunctionSioInput, FunctionSioOutput, Pin, Pins, PullDown};
use rp2040_hal::{Sio, Timer, Watchdog, pac, Clock};
use rp2040_hal::gpio::bank0::Gpio21;
use rp2040_hal::gpio::Interrupt::EdgeHigh;
use rp2040_hal::pac::interrupt;
use sx127x_lora::{Interrupt, RadioMode};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

const XOSC_CRYSTAL_FREQ_HZ: u32 = 12_000_000;
const LORA_FREQUENCY_MHZ: i64 = 915;

type Dio0 = Pin<Gpio21, FunctionSioInput, PullDown>;

static DIO0: Mutex<RefCell<Option<Dio0>>> = Mutex::new(RefCell::new(None));
static DIO0_FLAG: AtomicBool = AtomicBool::new(false);

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let spi_mosi = pins.gpio15.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio8.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio14.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );
    let spi_bus = RefCell::new(spi);

    let mut led: Pin<_, FunctionSioOutput, PullDown> = pins.gpio13.reconfigure();
    let nss = pins.gpio16.into_push_pull_output_in_state(PinState::High);
    let reset = pins.gpio17.into_push_pull_output_in_state(PinState::High);

    let spi_device = RefCellDevice::new(&spi_bus, nss, timer).unwrap();
    let mut lora = sx127x_lora::LoRa::new(spi_device, reset, LORA_FREQUENCY_MHZ).unwrap();

    let dio0: Pin<Gpio21, FunctionSioInput, PullDown> = pins.gpio21.reconfigure();
    dio0.set_interrupt_enabled(EdgeHigh, true);

    critical_section::with(|cs| {
        DIO0.borrow(cs).replace(Some(dio0));
    });

    lora.enable_interrupt(Interrupt::RxDone).unwrap();
    lora.set_mode(RadioMode::RxContinuous).unwrap();
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        if DIO0_FLAG.load(Ordering::Relaxed) {
            lora.clear_interrupt(Interrupt::RxDone).unwrap();
            match lora.read_packet() {
                Ok(_) => {
                    led.set_high().unwrap();
                    timer.delay_ms(500);
                    led.set_low().unwrap();
                    timer.delay_ms(500);
                }
                Err(_) => led.set_high().unwrap()
            }
            DIO0_FLAG.store(false, Ordering::Relaxed);
        }

        wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    critical_section::with(|cs| {
        let mut maybe_dio0 = DIO0.borrow(cs).borrow_mut();
        if let Some(dio0) = maybe_dio0.as_mut() {
            DIO0_FLAG.store(true, Ordering::Relaxed);
            dio0.clear_interrupt(EdgeHigh);
        }
    })
}