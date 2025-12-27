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
The library currently supports the `TxDone` and `RxDone` interrupts on the `DIO0` pin. Blocking (e.g. `poll_irq(...)`)
and GPIO-interrupt-driven (e.g. `RxDone` on `DIO0`) approaches are available, and you can see examples of both in
`examples/rp2040`.

## Tests
From the root dir: `$ cargo test`

## TODO
* Add async support
* Support additional interrupts (e.g. `RxTimeout` on `DIO1`)

## Contributing
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.