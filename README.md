# sx127x_lora
[![](http://meritbadge.herokuapp.com/sx127x-lora)](https://crates.io/crates/sx127x-lora)
![](https://img.shields.io/hexpm/l/plug.svg)

 A platform-agnostic driver for Semtech SX1276/77/78/79 based boards. It supports any device that
implements the `embedded-hal` traits. Devices are connected over SPI and require an extra GPIO pin for
RESET. This crate works with any Semtech based board including:
 * Modtronix inAir4, inAir9, and inAir9B
 * HopeRF RFM95W, RFM96W, and RFM98W
 * Adafruit RP2040 RFM95

## Interrupts
The crate currently polls the IRQ register on the radio to determine if a new packet has arrived. This
would be more efficient if instead an interrupt was connected to the module's DIO_0 pin. Once interrupt
support is available in `embedded-hal`, then this will be added. It is possible to implement this function on a
device-to-device basis by retrieving a packet with the `read_packet()` function.

## TODO
* Implement DIO_0 interrupt
* Add async support

## Contributing
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.