//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The
//! sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt
//! functionality, lower power consumption, and a new FIFO functionality. The integrated 512 byte FIFO buffer supports
//! low power applications and prevents data loss in non-real-time systems.
//!
//! [`Bmp390`] is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the
//! sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal_async::i2c::I2c`]
//! trait to be compatible with a wide range of embedded platforms. Measurements utilize the [`uom`] crate to provide
//! automatic, type-safe, and zero-cost units of measurement for [`Measurement`].
//!
//! # Example
//! ```no_run
//! # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
//! use bmp390::dd::{Bmp390, Sdo, I2cInterface};
//! # async fn run() -> Result<(), embedded_hal_async::i2c::ErrorKind> {
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
//! let interface = I2cInterface {
//!     bus: i2c,
//!     address: Sdo::Up,
//! };
//!
//! let mut bmp390 = Bmp390::new(interface);
//!
//! // read individual registers
//! let chip_id = bmp390.device().chip_id().read_async().await?;
//! assert_eq!(chip_id.value(), 0x60);
//!
//! // read a calibrated measurement
//! let measurement = bmp390.measure_async().await?;
//! defmt::info!("Measurement: {}", measurement);
//! # Ok(())
//! # }
//! ```
//!
//! # Datasheet
//! The [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)
//! contains detailed information about the sensor's features, electrical characteristics, and registers. This package
//! implements the functionality described in the datasheet and references the relevant sections in the documentation.
//!
//! # Synchronous and Asynchronous API
//! The synchronous API uses the same driver as the asynchronous driver, but with
//! synchronous methods. The synchronous API is available any time the provided
//! bus interface implements the synchronous [`embedded_hal::i2c::I2c`] trait.
//! Asynchronous methods are typically suffixed in `_async`.

mod bmp390;
mod generated;
pub mod i2c;

pub use bmp390::Bmp390;
pub use generated::*;
pub use i2c::{I2cInterface, Sdo};
