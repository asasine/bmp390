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
//! use bmp390::{
//!     Bmp390,
//!     interfaces::{I2cInterface, Sdo},
//! };
//! # async fn run() -> Result<(), embedded_hal_async::i2c::ErrorKind> {
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
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

#![no_std]
#![cfg_attr(docsrs, feature(doc_cfg))]

mod bmp390;
mod extras;
#[rustfmt::skip]
mod generated;
pub mod interfaces;
mod measurement;
pub mod registers;

pub use bmp390::{Bmp390, ConfigurationBuilder};
pub use measurement::Measurement;

pub mod raw {
    //! Low-level register interface, provided by [`device_driver`].
    //!
    //! Most applications should use [`crate::Bmp390`] and the ergonomic high-level API it provides.
    //! These types are provided for when low-level manipulation of the sensor registers is
    //! necessary, or for when ergonomic deserialization fails for some reason (e.g., reserved
    //! values).
    pub use crate::generated::*;
}
