//! Receive packets using the LoRa 1276 module on the Adafruit Feather RP2040 RFM95 board.
//!
//! Output is available via serial over a USB-C connection, and panics will turn the on-board LED
//! on.

#![no_std]
#![no_main]

extern crate sx127x_lora;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::panic::PanicInfo;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal_bus::spi::RefCellDevice;
use rp2040_hal as hal;
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::gpio::{FunctionSioOutput, Pin, Pins, PullNone};
use rp2040_hal::{Sio, Timer, Watchdog, pac, Clock};
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::bank0::Gpio13;
use rp2040_hal::multicore::{Multicore, Stack};
use rp2040_hal::pac::{PPB, PSM};
use rp2040_hal::rtc::DateTime;
use rp2040_hal::sio::SioFifo;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use sx127x_lora::Sx127xError;
use crate::SioFifoMsg::*;

enum SioFifoMsg {
    ErrReceiving = 0x0,
    ErrReset = 0x1,
    ErrSpi = 0x2,
    ErrTransmitting = 0x3,
    ErrUninformative = 0x4,
    ErrVersionMismatch = 0x5,
    PacketNotReady = 0x6,
    RxOk = 0x7,
    SetupDone = 0x8,
    StartPolling = 0x9,
}
impl TryFrom<u32> for SioFifoMsg {
    type Error = ();
    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(ErrReceiving),
            0x1 => Ok(ErrReset),
            0x2 => Ok(ErrSpi),
            0x3 => Ok(ErrTransmitting),
            0x4 => Ok(ErrUninformative),
            0x5 => Ok(ErrVersionMismatch),
            0x6 => Ok(PacketNotReady),
            0x7 => Ok(RxOk),
            0x8 => Ok(SetupDone),
            0x9 => Ok(StartPolling),
            _ => Err(())
        }
    }
}

impl From<SioFifoMsg> for &[u8] {
    fn from(value: SioFifoMsg) -> Self {
        match value {
            ErrReceiving => "LoRa RX err: receiving\r\n",
            ErrReset => "LoRa RX err: reset\r\n",
            ErrSpi => "LoRa RX err: SPI\r\n",
            ErrTransmitting => "LoRa RX err: transmitting\r\n",
            ErrUninformative => "LoRa RX err: uninformative\r\n",
            ErrVersionMismatch => "LoRa RX err: version mismatch\r\n",
            PacketNotReady => "LoRa RX: packet not ready\r\n",
            RxOk => "LoRa RX ok\r\n",
            SetupDone => "LoRa RX: setup done\r\n",
            StartPolling => "LoRa RX: start polling\r\n",
        }.as_bytes()
    }
}

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[unsafe(link_section = ".boot2")]
#[unsafe(no_mangle)]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static CORE1_STACK: Stack<4096> = Stack::new();

const XOSC_CRYSTAL_FREQ_HZ: u32 = 12_000_000;
const LORA_FREQUENCY_MHZ: i64 = 915;

static SIO_FIFO: Mutex<RefCell<Option<SioFifo>>> = Mutex::new(RefCell::new(None));
static LED_PIN: Mutex<RefCell<Option<Pin<Gpio13, FunctionSioOutput, PullNone>>>> = Mutex::new(RefCell::new(None));

fn core1_task(_sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
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

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Raspberry Pi")
            .product("RP2 USB Serial")
            .serial_number("E0C9125B0D9B")])
        .unwrap()
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        match sio.fifo.read() {
            Some(word) => {
                if let Ok(msg) = SioFifoMsg::try_from(word) {
                    serial.write(msg.into()).unwrap();
                } else {
                    serial.write("Unexpected msg\r\n".as_bytes()).unwrap();
                }
            },
            None => continue,
        }
    }
}

fn init_core_1(psm: &mut PSM, ppb: &mut PPB, fifo: &mut SioFifo, clk_freq: u32, ) {
    let mut mc = Multicore::new(psm, ppb, fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let sys_freq = clk_freq;
    let _ = core1.spawn(CORE1_STACK.take().unwrap(), move || core1_task(sys_freq));
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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
    let mut rtc = hal::rtc::RealTimeClock::new(
        pac.RTC,
        clocks.rtc_clock,
        &mut pac.RESETS,
        DateTime {
            year: 0,
            month: 1,
            day: 1,
            day_of_week: hal::rtc::DayOfWeek::Monday,
            hour: 0,
            minute: 0,
            second: 0,
        },
    )
        .unwrap();
    rtc.now().unwrap();

    init_core_1(
        &mut pac.PSM,
        &mut pac.PPB,
        &mut sio.fifo,
        clocks.system_clock.freq().to_Hz()
    );

    critical_section::with(|cs| {
        SIO_FIFO.borrow(cs).replace(Some(sio.fifo));
    });

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

    let nss = pins.gpio16.into_push_pull_output_in_state(PinState::High);
    let reset = pins.gpio17.into_push_pull_output_in_state(PinState::High);

    let spi_device = RefCellDevice::new(&spi_bus, nss, timer).unwrap();
    let mut lora = sx127x_lora::LoRa::new(spi_device, reset, LORA_FREQUENCY_MHZ).unwrap();

    timer.delay_ms(1000);
    send_sio_fifo_msg(SetupDone);

    loop {
        send_sio_fifo_msg(StartPolling);
        let msg = match lora.poll_irq(None) {
            Ok(_) => {
                // TODO handle packet_size > 255
                match lora.read_packet() {
                    Ok(_) => RxOk,
                    Err(e) => match e {
                        Sx127xError::Uninformative => ErrUninformative,
                        Sx127xError::VersionMismatch(_) => ErrVersionMismatch,
                        Sx127xError::Reset(_) => ErrReset,
                        Sx127xError::SPI(_) => ErrSpi,
                        Sx127xError::Transmitting => ErrTransmitting,
                        Sx127xError::Receiving => ErrReceiving,
                    },
                }
            },
            Err(e) => match e {
                Sx127xError::Uninformative => ErrUninformative,
                Sx127xError::VersionMismatch(_) => ErrVersionMismatch,
                Sx127xError::Reset(_) => ErrReset,
                Sx127xError::SPI(_) => ErrSpi,
                Sx127xError::Transmitting => ErrTransmitting,
                Sx127xError::Receiving => ErrReceiving,
            },
        };
        send_sio_fifo_msg(msg);
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    critical_section::with(|cs| {
        let mut maybe_led = LED_PIN.borrow(cs).borrow_mut();
        if let Some(led) = maybe_led.deref_mut() {
            led.set_high().unwrap();
        }
    });
    loop {}
}

fn send_sio_fifo_msg(msg: SioFifoMsg) {
    critical_section::with(|cs| {
        let mut maybe_sio_fifo = SIO_FIFO.borrow_ref_mut(cs);
        if let Some(sio_fifo) = maybe_sio_fifo.as_mut() {
            sio_fifo.write(msg as u32);
        }
    });
}