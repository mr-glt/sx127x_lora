# sx127x_lora
[![](http://meritbadge.herokuapp.com/sx127x-lora)](https://crates.io/crates/sx127x-lora)
![](https://img.shields.io/hexpm/l/plug.svg)

 A platform-agnostic driver for the Semtech SX1276/77/78/79 based boards. It supports any device that
implements the `embedded-hal` traits. Devices are connected over SPI and require an extra GPIO pin for
RESET. This cate works with any Semtech based board including:
 * Modtronix inAir4, inAir9, and inAir9B
 * HopeRF RFM95W, RFM96W, and RFM98W
# Examples
## Raspberry Pi Basic Send
Utilizes a Raspberry Pi to send a message. The example utilizes the `linux_embedded_hal` crate.
```rust
#![feature(extern_crate_item_prelude)]
extern crate sx127x_lora;
extern crate linux_embedded_hal as hal;

use hal::spidev::{self, SpidevOptions};
use hal::{Pin, Spidev};
use hal::sysfs_gpio::Direction;
use hal::Delay;

const LORA_CS_PIN: u64 = 8;
const LORA_RESET_PIN: u64 = 21;
const FREQUENCY: i64 = 915;

fn main(){

    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(20_000)
        .mode(spidev::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let cs = Pin::new(LORA_CS_PIN);
    cs.export().unwrap();
    cs.set_direction(Direction::Out).unwrap();

    let reset = Pin::new(LORA_RESET_PIN);
    reset.export().unwrap();
    reset.set_direction(Direction::Out).unwrap();

    let mut lora = sx127x_lora::LoRa::new(
        spi, cs, reset,  FREQUENCY, Delay)
        .expect("Failed to communicate with radio module!");

    lora.set_tx_power(17,1); //Using PA_BOOST. See your board for correct pin.

    let message = "Hello, world!";
    let mut buffer = [0;255];
    for (i,c) in message.chars().enumerate() {
        buffer[i] = c as u8;
    }

    let transmit = lora.transmit_payload(buffer,message.len());
    match transmit {
        Ok(packet_size) => println!("Sent packet with size: {}", packet_size),
        Err(()) => println!("Error"),
    }
}
```
## STM32F429 Blocking Receive
Utilizes a STM32F429 to receive data using the blocking `poll_irq(timeout)` function. It prints
the received packet back out over semihosting. The example utilizes the `stm32f429_hal`, `cortex_m`,
and `panic_semihosting` crates.
```rust
#![no_std]
#![no_main]

extern crate sx127x_lora;
extern crate stm32f429_hal as hal;
extern crate cortex_m;
extern crate panic_semihosting;

use sx127x_lora::MODE;
use cortex_m_semihosting::*;
use hal::gpio::GpioExt;
use hal::flash::FlashExt;
use hal::rcc::RccExt;
use hal::time::MegaHertz;
use hal::spi::Spi;
use hal::delay::Delay;

const FREQUENCY: i64 = 915;

#[entry]
fn main() -> !{
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
    let reset = gpiof.pf13.into_push_pull_output(&mut gpiof.moder, &mut gpiof.otyper);
    let cs = gpiod.pd14.into_push_pull_output(&mut gpiod.moder, &mut gpiod.otyper);

    let spi = Spi::spi1(
        p.SPI1,
        (sck, miso, mosi),
        MODE,
        MegaHertz(8),
        clocks,
        &mut rcc.apb2,
    );

    let mut lora = sx127x_lora::LoRa::new(
        spi, cs, reset, FREQUENCY,
        Delay::new(cp.SYST, clocks)).unwrap();

    loop {
        let poll = lora.poll_irq(Some(30)); //30 Second timeout
        match poll {
            Ok(size) =>{
               hprint!("with Payload: ");
               let buffer = lora.read_packet(); // Received buffer. NOTE: 255 bytes are always returned
               for i in 0..size{
                   hprint!("{}",buffer[i] as char).unwrap();
               }
               hprintln!();
            },
            Err(()) => hprintln!("Timeout").unwrap(),
        }
    }
}
```
## Interrupts
The crate currently polls the IRQ register on the radio to determine if a new packet has arrived. This
would be more efficient if instead an interrupt was connect the the module's DIO_0 pin. Once interrupt
support is available in `embedded-hal`, then this will be added. It is possible to implement this function on a
device-to-device basis by retrieving a packet with the `read_packet()` function.

## Contributing
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
