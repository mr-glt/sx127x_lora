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

    let message = b"Hello, world!";

    let transmit = lora.transmit_payload(message);
    match transmit {
        Ok(_) => println!("Sent packet"),
        Err(e) => println!("Error: {:?}", e),
    }
}