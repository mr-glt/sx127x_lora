# RP2040 Examples

The RP2040 examples are based upon an [Adafruit Feather RP2040 RFM95](https://www.adafruit.com/product/5714).

### Usage

From this directory:

1. Attach RP2040 feather target to host machine in boot mode.
2. Flash firmware, e.g.: `$ cargo run --example tx`.
3. View output via serial client on host, e.g.: `minicom -D /dev/ttyACM0 -b 115200`.