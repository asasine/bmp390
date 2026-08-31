//! Private module containing the [`Bmp390`] driver implementation.
//!
//! [`Bmp390`] is re-exported by the crate's root module.
//!
//! # Module organization
//! - [`measure`]: methods for retrieving measurements from the sensor, including calibration data.
//! - [`configuration_builder`]: methods for building and applying device configurations.

mod configuration_builder;
mod measure;
#[cfg(test)]
mod test_utils;

pub use configuration_builder::ConfigurationBuilder;

use crate::{raw::Device, registers::Coefficients};
use uom::si::{f32::Length, length::meter};

/// A driver for the BMP390 pressure sensor over multiple bus implementations.
///
/// To use this driver, create a new instance with [`Bmp390::new`]. Then, use [`Bmp390::device`] to get
/// a [`Device`] instance, which has methods for reading and writing individual registers.
/// For higher-level functionality, use methods on [`Bmp390`] to retrieve calibrated measurements, retrieve unit-safe
/// values, and configure the device.
///
/// # Multi-bus support
/// This driver supports multiple bus implementations, including synchronous and asynchronous I2C.
/// This is achieved through the [`device_driver`] crate, which provides a common interface for reading/writing
/// registers and sending commands. Any bus implementation provided to [`Bmp390::new`] that implements the
/// [`device_driver`] interface traits can be used with this driver.
///
/// # Safe units
/// Unit-safe measurements can be retrieved with [`Bmp390::measure`], which returns a [`Measurement`] struct containing
/// the pressure, temperature, and altitude.
/// This driver utilizes [`uom`] to provide automatic, type-safe, and zero-cost units of measurement. The altitude is
/// calculated based on the current pressure, standard atmospheric pressure at sea level,
/// and a reference altitude, which can be set with [`Bmp390::set_reference_altitude`]. The reference altitude defaults
/// to zero, so the default altitude is measured from sea level.
///
/// # Example
/// ```no_run
/// use bmp390::{
///     Bmp390,
///     interfaces::{I2cInterface, Sdo},
/// };
/// # async fn run() -> Result<(), embedded_hal_async::i2c::ErrorKind> {
/// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// let interface = I2cInterface {
///     bus: i2c,
///     address: Sdo::Up,
/// };
///
/// let mut bmp390 = Bmp390::new(interface);
///
/// // read individual registers
/// let chip_id = bmp390.device().chip_id().read_async().await?;
/// assert_eq!(chip_id.value(), 0x60);
///
/// // read a calibrated measurement
/// let measurement = bmp390.measure_async().await?;
/// defmt::info!("Measurement: {}", measurement);
/// # Ok(())
/// # }
/// ```
#[derive(Debug)]
pub struct Bmp390<I> {
    /// The device's register and command interface.
    device: Device<I>,

    /// The calibration coefficients read from the device's non-volatile memory.
    coefficients: Option<Coefficients>,

    /// The reference altitude used for calculating the altitude from the pressure measurement.
    ///
    /// By default, this is zero, set to the standard atmospheric pressure at sea level, 1013.25 hPa. It can be set to
    /// a different value using [`Self::set_reference_altitude`] to calculate the altitude relative to a different
    /// reference point.
    reference_altitude: Length,
}

#[cfg(feature = "defmt")]
impl<I: defmt::Format> defmt::Format for Bmp390<I> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Bmp390 {{ interface: {}, coefficients: {}, reference_altitude: {} m }}",
            self.device.interface,
            self.coefficients,
            self.reference_altitude.get::<meter>()
        );
    }
}

impl<I> Bmp390<I> {
    /// Create a new [`Bmp390`] from a device interface.
    pub fn new(interface: I) -> Self {
        Self {
            device: Device::new(interface),
            coefficients: None,
            reference_altitude: Length::new::<meter>(0.0),
        }
    }

    /// Get a [`Device`] instance, which can be used to read and write individual registers on the device.
    pub fn device(&mut self) -> &mut Device<I> {
        &mut self.device
    }
}
