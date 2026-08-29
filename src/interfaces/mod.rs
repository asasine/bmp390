//! Interfaces for using the [`Bmp390`] device driver with different communication protocols.
//!
//! - I2C: [`I2cInterface`] implements the [`device_driver`] interface traits for
//!   reading and writing registers over I2C.
//! - Commands: [`Polling`] wraps any interface and invokes commands by polling
//!   the device for command completion.
//!
//! [`Bmp390`]: crate::Bmp390

mod i2c;
mod polling;

pub use i2c::{I2cInterface, Sdo};
pub use polling::{CommandError, Polling};
