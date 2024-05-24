# BMP390

[![Crates.io Version](https://img.shields.io/crates/v/bmp390?logo=rust)](https://crates.io/crates/bmp390)
[![Docs](https://docs.rs/bmp390/badge.svg)](https://docs.rs/bmp390)
[![CI](https://img.shields.io/github/actions/workflow/status/asasine/bmp390/rust.yaml?branch=main&logo=github&label=CI)](https://github.com/asasine/bmp390/actions/workflows/rust.yaml?query=branch%3Amain)
[![Crates.io Downloads](https://img.shields.io/crates/d/bmp390)](https://crates.io/crates/bmp390)

The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt functionality, lower power, and a FIFO functionality. The integrated 512 byte FIFO buffer supports low power applications and prevents data loss in non-real-time systems.

[`Bmp390`](https://docs.rs/bmp390/latest/bmp390/struct.Bmp390.html) is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal_async::i2c`](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/index.html) traits to be compatible with a wide range of embedded platforms.

## Datasheet
The [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) contains detailed information about the sensor's features, electrical characteristics, and registers. This package implements the functionality described in the datasheet and references the relevant sections in the documentation.
