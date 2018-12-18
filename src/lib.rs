#![allow(non_snake_case)]
#![allow(dead_code)]
#![allow(unused_assignments)]
#![crate_type = "lib"]
#![crate_name = "sx127x_lora"]

//! # sx127x_lora
//! The _sx127x_lora_ crate provides an interface to Semtech SX1276/77/78/79 based boards. Due to
//! its dependency on the `rppal` crate, the library currently only works on Raspberry Pi. It
//! requires that both the SPI and GPIO interfaces are enabled on the Pi. This can accomplished with
//! `sudo raspi-config` and a reboot. __This library is under heavy development is subject to changes__
//! # Hookup
//! ### Radio --------> Raspberry Pi
//! #### 3V -----------> 3V
//! #### GND ----------> GND
//! #### MOSI ---------> MOSI/BCM 10/GPIO 19
//! #### MISO ---------> MISO/BCM 9/GPIO 21
//! #### SCK ----------> SCLK/BCM 11/GPIO 23
//! #### RESET --------> Any Open GPIO
//! #### DIO_0 --------> Any Open GPIO
//!
//! # Examples
//! ## Basic send
//! ```no_run
//! extern crate sx127x_lora;
//! const LORA_CS_PIN: u8 = 8;
//! const DIO_0_PIN: u8 = 20;
//! const LORA_RESET_PIN: u8 = 21;
//! const FREQUENCY: i64 = 915;
//! fn main(){
//!     let mut lora = sx127x_lora::LoRa::new(LORA_CS_PIN,LORA_RESET_PIN,
//!                                           DIO_0_PIN,FREQUENCY).unwrap();
//!     lora.set_tx_power(17,1); //Using PA_BOOST. See your board for correct pin.
//!     let transmit = lora.transmit_string("Hello World!".to_string());
//!     match transmit {
//!         Ok(packet_size) => println!("Sent packet with size: {}", packet_size),
//!         Err(e) => println!("Error: {}", e),
//!     }
//! }
//! ```
//! ## Asynchronous send and receive
//! Utilizes a `Arc<Mutex<sx127x_lora::LoRa>>` to allow for an asynchronous interrupt to be handled
//! when a new packet is received. The `lazy_static` crate is used to create a global static ref.
//! The example is utilizes the `rppal` crate directly to setup the interrupt pin. This will change
//! in the future. Make sure to add `lazy_static = "1.2.0"` under `[dependencies]` in your Cargo.toml.
//! ```no_run
//! #[macro_use]
//! extern crate lazy_static;
//! extern crate sx127x_lora;
//!
//! use rppal::gpio::{Level,Trigger};
//! use std::time::Duration;
//! use std::{thread};
//! use std::sync::{Arc, Mutex};
//!
//! const LORA_CS_PIN: u8 = 8;
//! const DIO_0_PIN: u8 = 20;
//! const LORA_RESET_PIN: u8 = 21;
//! const FREQUENCY: i64 = 915;
//!
//! lazy_static! {
//!     static ref LORA: Arc<Mutex<sx127x_lora::LoRa>> = Arc::new(Mutex::new(sx127x_lora::LoRa::
//!         new(LORA_CS_PIN,LORA_RESET_PIN, DIO_0_PIN,FREQUENCY).unwrap()));
//! }
//!
//! fn main() {
//!     let lora_clone = LORA.clone();
//!     let mut lora = lora_clone.lock().unwrap();
//!     lora.set_tx_power(17,1);
//!     lora.gpio.set_async_interrupt(DIO_0_PIN,Trigger::RisingEdge,handle_new_packet);
//!     lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
//!     drop(lora);
//!     loop{
//!         let mut lora = lora_clone.lock().unwrap();
//!         let transmit = lora.transmit_string("Hello world!".to_string());
//!         match transmit {
//!             Ok(s) => println!("Sent packet with size: {}", s),
//!             Err(e) => println!("Error: {}",e),
//!         }
//!         lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
//!         drop(lora);
//!         thread::sleep(Duration::from_millis(5000));
//!     }
//! }
//!
//! fn handle_new_packet(_level: Level){
//!     let lora_clone = LORA.clone();
//!     let mut lora = lora_clone.lock().unwrap();
//!     println!("New Packet with rssi: {} and SNR: {}",lora.get_packet_rssi(),
//!     lora.get_packet_snr());
//!     let buffer = lora.read_packet();
//!     print!("Payload: ");
//!     for i in buffer.iter() {
//!         print!("{}",*i as char);
//!     }
//!     println!();
//!     lora.set_mode(sx127x_lora::RadioMode::RxContinuous);
//!     drop(lora);
//! }
//!```
extern crate spidev;
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SPI_MODE_0};
use rppal::gpio::{Gpio, Mode, Level};
use std::io;
use std::time::Duration;
use std::{thread};

// registers
const REG_FIFO: u8 = 0x00;
const REG_OP_MODE: u8 = 0x01;
const REG_FRF_MSB: u8 = 0x06;
const REG_FRF_MID: u8 = 0x07;
const REG_FRF_LSB: u8 = 0x08;
const REG_PA_CONFIG: u8 = 0x09;
const REG_OCP: u8 = 0x0b;
const REG_LNA: u8 = 0x0c;
const REG_FIFO_ADDR_PTR: u8 = 0x0d;
const REG_FIFO_TX_BASE_ADDR: u8 = 0x0e;
const REG_FIFO_RX_BASE_ADDR: u8 = 0x0f;
const REG_FIFO_RX_CURRENT_ADDR: u8 = 0x10;
const REG_IRQ_FLAGS: u8 = 0x12;
const REG_RX_NB_BYTES: u8 = 0x13;
const REG_PKT_SNR_VALUE: u8 = 0x19;
const REG_PKT_RSSI_VALUE: u8 = 0x1a;
const REG_MODEM_CONFIG_1: u8 = 0x1d;
const REG_MODEM_CONFIG_2: u8 = 0x1e;
const REG_PREAMBLE_MSB: u8 = 0x20;
const REG_PREAMBLE_LSB: u8 = 0x21;
const REG_PAYLOAD_LENGTH: u8 = 0x22;
const REG_MODEM_CONFIG_3: u8 = 0x26;
const REG_FREQ_ERROR_MSB: u8 = 0x28;
const REG_FREQ_ERROR_MID: u8 = 0x29;
const REG_FREQ_ERROR_LSB: u8 = 0x2a;
const REG_RSSI_WIDEBAND: u8 = 0x2c;
const REG_DETECTION_OPTIMIZE: u8 = 0x31;
const REG_INVERTIQ: u8 = 0x33;
const REG_DETECTION_THRESHOLD: u8 = 0x37;
const REG_SYNC_WORD: u8  = 0x39;
const REG_INVERTIQ2: u8 = 0x3b;
const REG_DIO_MAPPING_1: u8 = 0x40;
const REG_VERSION: u8 = 0x42;
const REG_PA_DAC: u8 = 0x4d;

// modes
const MODE_LONG_RANGE_MODE: u8 = 0x80;
const MODE_SLEEP: u8 = 0x00;
const MODE_STDBY: u8 = 0x01;
const MODE_TX: u8 = 0x03;
const MODE_RX_CONTINUOUS: u8 = 0x05;
const MODE_RX_SINGLE: u8 = 0x06;

// PA config
const PA_BOOST: u8 = 0x80;
const PA_OUTPUT_RFO_PIN: u8 = 0;

// IRQ masks
const IRQ_TX_DONE_MASK: u8 = 0x08;
const IRQ_PAYLOAD_CRC_ERROR_MASK: u8 = 0x20;
const IRQ_RX_DONE_MASK: u8 = 0x40;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_radio() {
        let lora = LoRa::new(8,21, 20,915);
        assert_eq!(lora.is_ok(), true);
    }
}

/// Provides high-level access to Semtech SX1276/77/78/79 based boards connected to a Raspberry Pi
pub struct LoRa{
    pub initialized: bool,
    cs_pin: u8,
    reset_pin: u8,
    pub dio_0_pin: u8,
    frequency: i64,
    pub spi: Spidev,
    pub gpio: Gpio,
    pub explicitHeader: bool,
    pub mode: RadioMode,
}

///Radio states
pub enum RadioMode{
    RxSingle,
    RxContinuous,
    Tx,
    Sleep,
    Standby,
}

impl LoRa{
    /// Constructs a new `LoRa`.
    ///
    /// Only one instance should exist at a time. Additional instances of
    /// `LoRa` should be created by through cloning `Arc<Mutex<lora::LoRa>>`,
    /// or channels. When created, the radio is reset, frequency is set and
    /// the radio is put into `Standby`.
    pub fn new(cs_pin: u8, reset_pin: u8, dio_0_pin: u8, frequency: i64) -> Result<LoRa,()>{
        let spi = LoRa::create_spi();
        match spi {
            Ok(spi) => {
                let mut gpio = Gpio::new().unwrap();
                gpio.set_mode(reset_pin, Mode::Output);
                gpio.set_mode(cs_pin, Mode::Output);
                gpio.write(cs_pin,Level::High);
                gpio.write(reset_pin,Level::Low);
                thread::sleep(Duration::from_millis(10));
                gpio.write(reset_pin,Level::High);
                thread::sleep(Duration::from_millis(10));
                let mut lora = LoRa{
                    initialized: true,
                    cs_pin,
                    reset_pin,
                    dio_0_pin,
                    frequency,
                    spi,
                    gpio,
                    explicitHeader: true,
                    mode: RadioMode::Sleep,
                };
                if LoRa::read_register(&lora,REG_VERSION) == 0x12 {
                    LoRa::set_mode(&mut lora,RadioMode::Sleep);
                    LoRa::set_frequency(&lora,frequency);
                    LoRa::write_register(&lora,REG_FIFO_TX_BASE_ADDR,0);
                    LoRa::write_register(&lora,REG_FIFO_RX_BASE_ADDR,0);
                    let val = LoRa::read_register(&lora,REG_LNA);
                    LoRa::write_register(&lora,REG_LNA, val | 0x03);
                    LoRa::write_register(&lora,REG_MODEM_CONFIG_3, 0x04);
                    LoRa::set_mode(&mut lora,RadioMode::Standby);
                    Ok(lora)
                }else{
                    Err(())
                }
            },
            Err(_e) => Err(()),
        }
    }

    /// Sets the state of the radio. Default mode after initiation is `Standby`.
    pub fn set_mode(&mut self,mode: RadioMode){
        if self.explicitHeader {
            LoRa::set_explicit_header_mode(self);
        }else{
            LoRa::set_implicit_header_mode(self);
        }
        match mode {
            RadioMode::RxSingle =>
                LoRa::write_register(&self,REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_RX_SINGLE),
            RadioMode::RxContinuous =>
                LoRa::write_register(&self,REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS),
            RadioMode::Tx =>
                LoRa::write_register(&self,REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_TX),
            RadioMode::Sleep =>
                LoRa::write_register(&self,REG_OP_MODE,MODE_LONG_RANGE_MODE | MODE_SLEEP),
            RadioMode::Standby =>
                LoRa::write_register(&self,REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY),
        }
        self.mode = mode;
    }

    /// Writes a string to the radio's FIFO and then waits till the radio is
    /// finished transmitting. The string must not be longer than 255 bytes.
    pub fn transmit_string(&mut self, value: String) -> Result<u8,String>{
        if value.len() <= 255 {
            if LoRa::is_transmitting(&self) {
                Err("Radio is already transmitting".to_string())
            }else{
                LoRa::set_mode(self,RadioMode::Standby);
                if self.explicitHeader {
                    LoRa::set_explicit_header_mode(self);
                }else{
                    LoRa::set_implicit_header_mode(self);
                }
                LoRa::write_register(&self,REG_FIFO_ADDR_PTR, 0);
                LoRa::write_register(&self,REG_PAYLOAD_LENGTH, 0);
                //Write Data
                let mut size = 0;
                for c in value.chars(){
                    LoRa::write_register(&self, REG_FIFO, c as u8);
                    size = size + 1;
                }
                LoRa::write_register(&self, REG_PAYLOAD_LENGTH,size);
                LoRa::set_mode(self, RadioMode::Tx);
                while LoRa::read_register(&self, REG_IRQ_FLAGS) !=0x8 {
                    thread::sleep(Duration::from_millis(100));
                }
                LoRa::write_register(&self,REG_IRQ_FLAGS, 0);
                Ok(size)
            }
        }else{
            Err("Packet too large".to_string())
        }
    }

    /// Writes a buffer of data to the radio's FIFO and then waits till the radio
    /// is finished transmitting. The buffer must not be longer than 255 bytes.
    pub fn transmit_payload(&mut self, buffer: Vec<u8>) -> Result<u8,String>{
        if buffer.len() <= 255 {
            if LoRa::is_transmitting(&self) {
                Err("Radio is already transmitting".to_string())
            }else{
                LoRa::set_mode(self,RadioMode::Standby);
                if self.explicitHeader {
                    LoRa::set_explicit_header_mode(self);
                }else{
                    LoRa::set_implicit_header_mode(self);
                }
                LoRa::write_register(&self,REG_FIFO_ADDR_PTR, 0);
                LoRa::write_register(&self,REG_PAYLOAD_LENGTH, 0);
                //Write Data
                let mut size = 0;
                for b in buffer{
                    LoRa::write_register(&self, REG_FIFO, b);
                    size = size + 1;
                }
                LoRa::write_register(&self, REG_PAYLOAD_LENGTH,size);
                LoRa::set_mode(self,RadioMode::Tx);
                while LoRa::read_register(&self, REG_IRQ_FLAGS) !=0x8 {
                    thread::sleep(Duration::from_millis(100));
                }
                LoRa::write_register(&self,REG_IRQ_FLAGS, 0);
                Ok(size)
            }
        }else {
            Err("Packet too large".to_string())
        }
    }

    /// Returns true if the radio is currently transmitting a packet.
    pub fn is_transmitting(&self) -> bool {
        if (LoRa::read_register(&self,REG_OP_MODE) & MODE_TX) == MODE_TX {
            true
        }else{
            if (LoRa::read_register(&self,REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 1{
                LoRa::write_register(&self,REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
            }
            false
        }
    }

    /// Clears the radio's IRQ registers.
    pub fn clear_irq(&self){
        let irq_flags = LoRa::read_register(&self,REG_IRQ_FLAGS);
        LoRa::write_register(&self,REG_IRQ_FLAGS, irq_flags);
    }

    /// Reads a packet from the the FIFO into a `Vec<u8>` and returns it. It also
    /// resets FIFO pointers and payload length registers.
    pub fn read_packet(&self) -> Vec<u8>{
        let mut buffer = Vec::new();
        let size = LoRa::read_register(&self, REG_RX_NB_BYTES);
        let fifo_addr = LoRa::read_register(&self, REG_FIFO_RX_CURRENT_ADDR);
        LoRa::write_register(&self, REG_FIFO_ADDR_PTR, fifo_addr);
        for _i in 0..size {
            let byte = LoRa::read_register(&self, REG_FIFO);
            buffer.push(byte);
            thread::sleep(Duration::from_millis(10));
        }
        LoRa::write_register(&self, REG_FIFO_ADDR_PTR, 0);
        LoRa::clear_irq(&self);
        buffer
    }

    /// Sets the transmit power and pin. Levels can range from 0-14 when the output
    /// pin = 0(RFO), and form 0-20 when output pin = 1(PA_BOOST). Power is in dB.
    /// Default value is `17`.
    pub fn set_tx_power(&self, mut level: i32, output_pin: u8){
        if PA_OUTPUT_RFO_PIN == output_pin {
            // RFO
            if level < 0 {
                level = 0;
            } else if level > 14 {
                level = 14;
            }
            LoRa::write_register(&self, REG_PA_CONFIG, (0x70 | level) as u8);
        } else {
            // PA BOOST
            if level > 17 {
                if level > 20 {
                    level = 20;
                }
                // subtract 3 from level, so 18 - 20 maps to 15 - 17
                level -= 3;

                // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
                LoRa::write_register(&self,REG_PA_DAC, 0x87);
                LoRa::set_ocp(&self,140);
            }else {
                if level < 2 {
                    level = 2;
                }
                //Default value PA_HF/LF or +17dBm
                LoRa::write_register(&self,REG_PA_DAC, 0x84);
                LoRa::set_ocp(&self,100);
            }
            level = level - 2;
            LoRa::write_register(&self,REG_PA_CONFIG, PA_BOOST | level as u8);
        }
    }

    /// Sets the over current protection on the radio(mA).
    pub fn set_ocp(&self, mA: u8){
        let mut ocp_trim: u8 = 27;

        if mA <= 120 {
            ocp_trim = (mA - 45) / 5;
        } else if mA <=240 {
            ocp_trim = (mA + 30) / 10;
        }
        LoRa::write_register(&self,REG_OCP, 0x20 | (0x1F & ocp_trim));
    }

    /// Sets the frequency of the radio. Values are in megahertz.
    /// I.E. 915 MHz must be used for North America. Check regulation for your area.
    pub fn set_frequency(&self, freq: i64){
        // calculate register values
        let base = 1;
        let FRF = (freq * (base << 19)) / 32;
        // write registers
        LoRa::write_register(&self,REG_FRF_MSB, ((FRF & 0xFF0000) >> 16) as u8);
        LoRa::write_register(&self,REG_FRF_MID, ((FRF & 0x00FF00) >> 8) as u8);
        LoRa::write_register(&self,REG_FRF_LSB, (FRF & 0x0000FF) as u8);
    }

    /// Sets the spreading factor of the radio. Supported values are between 6 and 12.
    /// If a spreading factor of 6 is set, implicit header mode must be used to transmit
    /// and receive packets. Default value is `7`.
    pub fn set_spreading_factor(&self,mut sf: u8) {
        if sf < 6 {
            sf = 6;
        } else if sf > 12 {
            sf = 12;
        }

        if sf == 6 {
            LoRa::write_register(&self,REG_DETECTION_OPTIMIZE, 0xc5);
            LoRa::write_register(&self,REG_DETECTION_THRESHOLD, 0x0c);
        } else {
            LoRa::write_register(&self,REG_DETECTION_OPTIMIZE, 0xc3);
            LoRa::write_register(&self,REG_DETECTION_THRESHOLD, 0x0a);
        }
        let modem_config_2 = LoRa::read_register(&self,REG_MODEM_CONFIG_2);
        LoRa::write_register(&self,REG_MODEM_CONFIG_2, (modem_config_2 & 0x0f) | ((sf << 4) & 0xf0));
        LoRa::set_ldo_flag(&self);
    }

    /// Sets the signal bandwidth of the radio. Supported values are: `7800 Hz`, `10400 Hz`,
    /// `15600 Hz`, `20800 Hz`, `31250 Hz`,`41700 Hz` ,`62500 Hz`,`125000 Hz` and `250000 Hz`
    /// Default value is `125000 Hz`
    pub fn set_signal_bandwidth(&self, sbw: i64){
        let bw: i64;
        match sbw {
            7800 => bw = 0,
            10400 => bw = 1,
            15600 => bw = 2,
            20800 => bw = 3,
            31250 => bw = 4,
            41700 => bw = 5,
            62500 => bw = 6,
            125000 => bw = 7,
            250000 => bw = 8,
            _ => bw = 9,
        }
        let modem_config_1 = LoRa::read_register(&self,REG_MODEM_CONFIG_1);
        LoRa::write_register(&self,REG_MODEM_CONFIG_1, (modem_config_1 & 0x0f) | ((bw << 4) as u8));
        LoRa::set_ldo_flag(&self);
    }

    /// Sets the coding rate of the radio with the numerator fixed at 4. Supported values
    /// are between `5` and `8`, these correspond to coding rates of `4/5` and `4/8`.
    /// Default value is `5`.
    pub fn set_coding_rate_4(&self, mut denominator: u8) {
        if denominator < 5 {
            denominator = 5;
        } else if denominator > 8 {
            denominator = 8;
        }
        let cr = denominator - 4;
        let modem_config_1 = LoRa::read_register(&self,REG_MODEM_CONFIG_1);
        LoRa::write_register(&self, REG_MODEM_CONFIG_1, (modem_config_1 & 0xf1) | (cr << 1));
    }

    /// Sets the preamble length of the radio. Values are between 6 and 65535.
    /// Default value is `8`.
    pub fn set_preamble_length(&self,length: i64) {
        LoRa::write_register(&self,REG_PREAMBLE_MSB, (length >> 8) as u8);
        LoRa::write_register(&self,REG_PREAMBLE_LSB, (length >> 0) as u8);
    }

    /// Enables are disables the radio's CRC check. Default value is `false`.
    pub fn set_crc(&self, value: bool){
        let modem_config_2 = LoRa::read_register(&self,REG_MODEM_CONFIG_2);
        if value {
            LoRa::write_register(&self,REG_MODEM_CONFIG_2, modem_config_2 | 0x04);
        }else{
            LoRa::write_register(&self,REG_MODEM_CONFIG_2, modem_config_2 & 0xfb);
        }
    }

    /// Inverts the radio's IQ signals. Default value is `false`.
    pub fn set_invert_iq(&self, value: bool) {
        if value {
            LoRa::write_register(&self,REG_INVERTIQ,  0x66);
            LoRa::write_register(&self,REG_INVERTIQ2, 0x19);
        }else{
            LoRa::write_register(&self,REG_INVERTIQ,  0x27);
            LoRa::write_register(&self,REG_INVERTIQ2, 0x1d);
        }
    }

    /// Sets the radio to use an explicit header. Default state is `ON`.
    pub fn set_explicit_header_mode(&mut self) {
        let reg_modem_config_1 = LoRa::read_register(&self,REG_MODEM_CONFIG_1);
        LoRa::write_register(&self,REG_MODEM_CONFIG_1,reg_modem_config_1  & 0xfe);
        self.explicitHeader = true;
    }

    /// Sets the radio to use an implicit header. Default state is `OFF`.
    pub fn set_implicit_header_mode(&mut self) {
        let reg_modem_config_1 = LoRa::read_register(&self,REG_MODEM_CONFIG_1);
        LoRa::write_register(&self,REG_MODEM_CONFIG_1,reg_modem_config_1  & 0x01);
        self.explicitHeader = false;
    }

    /// Returns the spreading factor of the radio.
    pub fn get_spreading_factor(&self) -> u8{
        LoRa::read_register(&self,REG_MODEM_CONFIG_2) >> 4
    }

    /// Returns the signal bandwidth of the radio.
    pub fn get_signal_bandwidth(&self) -> i64 {
        let bw = LoRa::read_register(&self,REG_MODEM_CONFIG_1) >> 4;
        match bw {
            0 => 7800,
            1 => 10400,
            2 => 15600,
            3 => 20800,
            4 => 312500,
            5 => 41700,
            6 => 62500,
            7 => 125000,
            8 => 250000,
            9 => 500000,
            _ => -1,
        }
    }

    /// Returns the RSSI of the last received packet.
    pub fn get_packet_rssi(&self) -> i32 {
        (LoRa::read_register(&self,REG_PKT_RSSI_VALUE) as i32) - 157
    }

    /// Returns the signal to noise radio of the the last received packet.
    pub fn get_packet_snr(&self) -> f64 {
        ((LoRa::read_register(&self,REG_PKT_SNR_VALUE) as f64) * 0.25)
    }

    /// Returns the frequency error of the last received packet in Hz.
    pub fn get_packet_frequency_error(&self) -> i64{
        let mut freq_error: i32 = 0;
        freq_error = (LoRa::read_register(&self, REG_FREQ_ERROR_MSB) & 0x7) as i32;
        freq_error <<= 8i64;
        freq_error += (LoRa::read_register(&self,REG_FREQ_ERROR_MID)) as i32;
        freq_error <<= 8i64;
        freq_error += (LoRa::read_register(&self, REG_FREQ_ERROR_LSB)) as i32;

        if LoRa::read_register(&self, REG_FREQ_ERROR_MSB) & 0x8 == 1 { // Sign bit is on
            freq_error -= 524288; // B1000'0000'0000'0000'0000
        }
        let f_xtal = 32000000; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
        let f_error = (((freq_error as f64) * (1i64 << 24) as f64) / f_xtal as f64) *
            (LoRa::get_signal_bandwidth(&self) as f64 / 500000.0f64); // p. 37
        (f_error as i64)
    }

    fn set_ldo_flag(&self) {
        let sw = LoRa::get_signal_bandwidth(&self);
        // Section 4.1.1.5
        let symbol_duration = 1000 / (sw / ((1 as i64) << LoRa::get_spreading_factor(&self))) ;

        // Section 4.1.1.6
        let ldo_on = symbol_duration > 16;

        let config_3 = LoRa::read_register(&self,REG_MODEM_CONFIG_3);
        LoRa::bit_write(config_3, 3, ldo_on);
        LoRa::write_register(&self,REG_MODEM_CONFIG_3, config_3);
    }

    fn write_register(&self, reg: u8, value: u8){
        let tx_buf = [reg | 0x80,value];
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            &self.spi.transfer(&mut transfer);
        }
    }

    fn read_register(&self,reg: u8) -> u8 {
        let tx_buf = [reg & 0x7f,0];
        let mut rx_buf = [0; 2];
        {
            let mut transfer = SpidevTransfer::read_write(&tx_buf, &mut rx_buf);
            &self.spi.transfer(&mut transfer);
        }
        rx_buf[1]
    }

    fn bit_write(num: u8, pos: u8, value: bool) -> u8{
        if (num >> pos) & (value as u8) != (value as u8) {
            let mask = 1 << pos;
            (num ^  mask)
        }else{
            num
        }
    }

    fn create_spi() -> io::Result<Spidev> {
        let mut spi = Spidev::open("/dev/spidev0.0")?;
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(20_000)
            .mode(SPI_MODE_0)
            .build();
        spi.configure(&options)?;
        Ok(spi)
    }
}
