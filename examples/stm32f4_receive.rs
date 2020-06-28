#![no_std]
#![no_main]

extern crate cortex_m;
extern crate panic_semihosting;
extern crate stm32f429_hal as hal;
extern crate sx127x_lora;

use cortex_m_semihosting::*;
use hal::delay::Delay;
use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use hal::spi::Spi;
use hal::time::MegaHertz;
use sx127x_lora::MODE;

const FREQUENCY: i64 = 915;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = hal::stm32f429::Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let mut flash = p.FLASH.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(MegaHertz(64))
        .pclk1(MegaHertz(32))
        .freeze(&mut flash.acr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb1);
    let mut gpiod = p.GPIOD.split(&mut rcc.ahb1);
    let mut gpiof = p.GPIOF.split(&mut rcc.ahb1);

    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let reset = gpiof
        .pf13
        .into_push_pull_output(&mut gpiof.moder, &mut gpiof.otyper);
    let cs = gpiod
        .pd14
        .into_push_pull_output(&mut gpiod.moder, &mut gpiod.otyper);

    let spi = Spi::spi1(
        p.SPI1,
        (sck, miso, mosi),
        MODE,
        MegaHertz(8),
        clocks,
        &mut rcc.apb2,
    );

    let mut lora =
        sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, Delay::new(cp.SYST, clocks)).unwrap();

    loop {
        let poll = lora.poll_irq(Some(30)); //30 Second timeout
        match poll {
            Ok(size) => {
                hprintln!(
                    "New Packet with size {} and RSSI: {}",
                    size,
                    lora.get_packet_rssi()
                )
                .unwrap();
                let buffer = lora.read_packet(); // Received buffer. NOTE: 255 bytes are always returned
                hprint!("with Payload: ");
                for i in 0..size {
                    hprint!("{}", buffer[i] as char).unwrap();
                }
                hprintln!();
            }
            Err(()) => hprintln!("Timeout").unwrap(),
        }
    }
}
