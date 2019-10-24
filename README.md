# SX127x LoRa Transmitter Rust Driver

This is a platform agnostic LoRa interface driver for the Semtech SX127x chips. This driver was built using [`embedded-hal`] traits and does not depend on std.

[`embedded-hal`]: https://docs.rs/embedded-hal/~0.2

By default the SX1276/7/8 family of chips is generated. To use the SX1272/3 family enable the *sx1272* feature in your cargo file.

## Usage

To use this driver, import this crate and an `embedded_hal` implementation, create the reset and chip select (NSS) pins and then instantiate the device. A partial example code could look like this:

```rust
let gpiochip = GpioChip::new(&"/dev/gpiochip0").unwrap();

let gpio_reset = gpiochip.request_line_values_output(&vec![RESET_GPIO], OutputMode::None, false, &"LoraHat reset").unwrap();
let gpio_nss = gpiochip.request_line_values_output(&vec![NSS_GPIO], OutputMode::None, false, &"LoraHat NSS").unwrap();

let spidev = SpiDev::new(&"/dev/spidev0.0").unwrap();
    
let lora_delay = Delay{};

println!("Initializing LoraHAT");

let mut sx1276 = SX1276::new(spidev,gpio_reset,gpio_nss, lora_delay).unwrap();

sx1276.set_transceiver_mode(TransceiverMode::Sleep).unwrap();

sx1276.set_frequency(&FREQUENCY).unwrap();
```

## Support

For questions, issues, feature requests, and other changes, please file an [issue in the github project](https://github.com/).

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

