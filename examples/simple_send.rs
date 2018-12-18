extern crate sx127x_lora;

const LORA_CS_PIN: u8 = 8;
const DIO_0_PIN: u8 = 20;
const LORA_RESET_PIN: u8 = 21;
const FREQUENCY: i64 = 915;

fn main(){
    let mut lora = sx127x_lora::LoRa::new(LORA_CS_PIN,LORA_RESET_PIN,
                                          DIO_0_PIN,FREQUENCY).unwrap();
    lora.set_tx_power(17,1); //Using PA_BOOST. See your board for correct pin.
    let transmit = lora.transmit_string("Hello World!".to_string());
    match transmit {
        Ok(packet_size) => println!("Sent packet with size: {}", packet_size),
        Err(e) => println!("Error: {}", e),
    }
}