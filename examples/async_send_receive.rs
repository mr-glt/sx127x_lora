#[macro_use]
extern crate lazy_static;
extern crate sx127x_lora;

use rppal::gpio::{Level,Trigger};
use std::time::Duration;
use std::{thread};
use std::sync::{Arc, Mutex};

const LORA_CS_PIN: u8 = 8;
const DIO_0_PIN: u8 = 20;
const LORA_RESET_PIN: u8 = 21;
const FREQUENCY: i64 = 915;

lazy_static! {
    static ref LORA: Arc<Mutex<sx127x_lora::LoRa>> = Arc::new(Mutex::new(sx127x_lora::LoRa::
        new(LORA_CS_PIN,LORA_RESET_PIN, DIO_0_PIN,FREQUENCY).unwrap()));
}

fn main() {
    let lora_clone = LORA.clone();
    let mut lora = lora_clone.lock().unwrap();
    lora.set_tx_power(17,1);
    lora.gpio.set_async_interrupt(DIO_0_PIN,Trigger::RisingEdge,handle_new_packet);
    lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
    drop(lora);
    loop{
        let mut lora = lora_clone.lock().unwrap();
        let transmit = lora.transmit_string("Hello world!".to_string());
        match transmit {
            Ok(s) => println!("Sent packet with size: {}", s),
            Err(e) => println!("Error: {}",e),
        }
        lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
        drop(lora);
        thread::sleep(Duration::from_millis(5000));
    }
}

fn handle_new_packet(_level: Level){
    let lora_clone = LORA.clone();
    let mut lora = lora_clone.lock().unwrap();
    println!("New Packet with rssi: {} and SNR: {}",lora.get_packet_rssi(), lora.get_packet_snr());
    let buffer = lora.read_packet();
    print!("Payload: ");
    for i in buffer.iter() {
        print!("{}",*i as char);
    }
    println!();
    lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
    drop(lora);
}