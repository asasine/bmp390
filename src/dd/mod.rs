//! A driver for the BMP390 barometric pressure and temperature sensor.
//!
//! This module contains a [`device_driver`]-based implementation of the BMP390 driver.

mod bmp390;
mod generated;

pub use generated::*;
pub use bmp390::{Bmp390, Sdo};
