use super::Device;
use crate::{
    CalibrationCoefficients, Measurement,
    dd::field_sets::{Config, FifoConfig, FifoWatermark, IfConf, IntCtrl, Odr, Osr, PwrCtrl},
};
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use thiserror::Error;
use uom::si::{
    f32::{Length, Pressure, ThermodynamicTemperature},
    length::meter,
};

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
/// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
/// use bmp390::dd::{Bmp390, Sdo, I2cInterface};
/// # async fn run() -> Result<(), embedded_hal_async::i2c::ErrorKind> {
/// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
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
    coefficients: Option<CalibrationCoefficients>,

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

    /// Set the reference altitude for altitude calculations.
    ///
    /// Following this, the altitude can be calculated using [`Self::altitude`]. If the current pressure matches
    /// the pressure when the reference altitude is set, the altitude will be 0.
    pub fn set_reference_altitude(&mut self, altitude: Length) {
        self.reference_altitude = altitude;
    }
}

/// A builder for configuring the BMP390 device.
///
/// By default, no registers are configured. Registers can be configured by calling
/// the corresponding methods on the builder. Once all desired registers are configured,
/// call [`Bmp390::configure`] to apply the configuration to the device.
pub struct ConfigurationBuilder {
    // registers
    fifo_watermark: Option<FifoWatermark>,
    fifo_config: Option<FifoConfig>,
    int_ctrl: Option<IntCtrl>,
    if_conf: Option<IfConf>,
    pwr_ctrl: Option<PwrCtrl>,
    osr: Option<Osr>,
    odr: Option<Odr>,
    config: Option<Config>,

    // Bmp390
    reference_altitude: Option<Length>,
}

impl ConfigurationBuilder {
    /// Create a new [`ConfigurationBuilder`] with no registers configured.
    pub const fn new() -> Self {
        Self {
            fifo_watermark: None,
            fifo_config: None,
            int_ctrl: None,
            if_conf: None,
            pwr_ctrl: None,
            osr: None,
            odr: None,
            config: None,
            reference_altitude: None,
        }
    }

    /// Set the FIFO watermark level.
    pub const fn fifo_watermark(mut self, fifo_watermark: FifoWatermark) -> Self {
        self.fifo_watermark = Some(fifo_watermark);
        self
    }

    /// Set the FIFO configuration.
    pub const fn fifo_config(mut self, fifo_config: FifoConfig) -> Self {
        self.fifo_config = Some(fifo_config);
        self
    }

    /// Set the interrupt control configuration.
    pub const fn int_ctrl(mut self, int_ctrl: IntCtrl) -> Self {
        self.int_ctrl = Some(int_ctrl);
        self
    }

    /// Set the serial interface settings.
    pub const fn if_conf(mut self, if_conf: IfConf) -> Self {
        self.if_conf = Some(if_conf);
        self
    }

    /// Set the power control configuration.
    pub const fn pwr_ctrl(mut self, pwr_ctrl: PwrCtrl) -> Self {
        self.pwr_ctrl = Some(pwr_ctrl);
        self
    }

    /// Set the oversampling settings.
    pub const fn osr(mut self, osr: Osr) -> Self {
        self.osr = Some(osr);
        self
    }

    /// Set the output data rate.
    pub const fn odr(mut self, odr: Odr) -> Self {
        self.odr = Some(odr);
        self
    }

    /// Set the IIR filter coefficients.
    pub const fn config(mut self, config: Config) -> Self {
        self.config = Some(config);
        self
    }

    /// Set the reference altitude for altitude calculations.
    ///
    /// This isn't a register itself, but is useful for updating the [`Bmp390`]
    /// during configuration.
    pub const fn reference_altitude(mut self, reference_altitude: Length) -> Self {
        self.reference_altitude = Some(reference_altitude);
        self
    }
}

impl Default for ConfigurationBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigureError<E> {
    #[error(transparent)]
    Interface(#[from] E),

    #[error("The configuration was invalid.")]
    InvalidConfiguration,
}

impl<I, E> Bmp390<I>
where
    I: AsyncRegisterInterface<AddressType = u8, Error = E>,
{
    /// Get the calibration coefficients, reading them from the device if they have not been read yet.
    async fn coefficients_async(&mut self) -> Result<&CalibrationCoefficients, E> {
        let coefficients = match self.coefficients.take() {
            Some(coefficients) => coefficients,
            None => self.device.calibration_data().read_async().await?.into(),
        };

        Ok(self.coefficients.insert(coefficients))
    }

    /// Configure the device with the given [`ConfigurationBuilder`].
    ///
    /// For each register that is set in the builder, the corresponding register
    /// will be written to the device.
    pub async fn configure_async(
        &mut self,
        builder: ConfigurationBuilder,
    ) -> Result<(), ConfigureError<E>> {
        let mut any = false;
        if let Some(fifo_watermark) = builder.fifo_watermark {
            self.device
                .fifo_watermark()
                .write_async(|w| *w = fifo_watermark)
                .await?;
            any = true;
        }

        if let Some(fifo_config) = builder.fifo_config {
            self.device
                .fifo_config()
                .write_async(|w| *w = fifo_config)
                .await?;
            any = true;
        }

        if let Some(int_ctrl) = builder.int_ctrl {
            self.device
                .int_ctrl()
                .write_async(|w| *w = int_ctrl)
                .await?;
            any = true;
        }

        if let Some(if_conf) = builder.if_conf {
            self.device.if_conf().write_async(|w| *w = if_conf).await?;
            any = true;
        }

        if let Some(pwr_ctrl) = builder.pwr_ctrl {
            self.device
                .pwr_ctrl()
                .write_async(|w| *w = pwr_ctrl)
                .await?;
            any = true;
        }

        if let Some(osr) = builder.osr {
            self.device.osr().write_async(|w| *w = osr).await?;
            any = true;
        }

        if let Some(odr) = builder.odr {
            self.device.odr().write_async(|w| *w = odr).await?;
            any = true;
        }

        if let Some(config) = builder.config {
            self.device.config().write_async(|w| *w = config).await?;
            any = true;
        }

        if any {
            let err_reg = self.device.err_reg().read_async().await?;
            if err_reg.conf_err() {
                return Err(ConfigureError::InvalidConfiguration);
            }
        }

        if let Some(reference_altitude) = builder.reference_altitude {
            self.set_reference_altitude(reference_altitude);
        }

        Ok(())
    }

    /// Get the calibrated temperature.
    pub async fn temperature_async(&mut self) -> Result<ThermodynamicTemperature, E> {
        let temperature = self.device.temperature_data().read_async().await?;
        let coefficients = self.coefficients_async().await?;
        Ok(coefficients.compensate_temperature(temperature.value()))
    }

    /// Get the calibrated temperature and pressure.
    pub async fn temperature_and_pressure_async(
        &mut self,
    ) -> Result<(ThermodynamicTemperature, Pressure), E> {
        let data = self.device.data().read_async().await?;
        let coefficients = self.coefficients_async().await?;
        let temperature = coefficients.compensate_temperature(data.temperature());
        let pressure = coefficients.compensate_pressure(temperature, data.pressure());
        Ok((temperature, pressure))
    }

    /// Get the calibrated pressure.
    ///
    /// Due to how the calibration works, this function also reads the temperature.
    pub async fn pressure_async(&mut self) -> Result<Pressure, E> {
        let (_, pressure) = self.temperature_and_pressure_async().await?;
        Ok(pressure)
    }

    /// Get the calibrated temperature, pressure, and altitude.
    ///
    /// The altitude is calculated based on the current pressure and the reference altitude.
    /// To change the altitude, use [`Self::set_reference_altitude`].
    pub async fn measure_async(&mut self) -> Result<Measurement, E> {
        let (temperature, pressure) = self.temperature_and_pressure_async().await?;
        Ok(Measurement {
            temperature,
            pressure,
            altitude: crate::calculate_altitude(pressure, self.reference_altitude),
        })
    }

    /// Get the altitude based on the current pressure and the reference altitude.
    ///
    /// To change the reference altitude, use [`Self::set_reference_altitude`].
    pub async fn altitude_async(&mut self) -> Result<Length, E> {
        let measurement = self.measure_async().await?;
        Ok(measurement.altitude)
    }
}

impl<I, E> Bmp390<I>
where
    I: RegisterInterface<AddressType = u8, Error = E>,
{
    /// Get the calibration coefficients, reading them from the device if they have not been read yet.
    fn coefficients(&mut self) -> Result<&CalibrationCoefficients, E> {
        let coefficients = match self.coefficients.take() {
            Some(coefficients) => coefficients,
            None => self.device.calibration_data().read()?.into(),
        };

        Ok(self.coefficients.insert(coefficients))
    }

    /// Configure the device with the given [`ConfigurationBuilder`].
    ///
    /// For each register that is set in the builder, the corresponding register
    /// will be written to the device.
    pub fn configure(&mut self, builder: ConfigurationBuilder) -> Result<(), ConfigureError<E>> {
        let mut any = false;
        if let Some(fifo_watermark) = builder.fifo_watermark {
            self.device
                .fifo_watermark()
                .write(|w| *w = fifo_watermark)?;
            any = true;
        }

        if let Some(fifo_config) = builder.fifo_config {
            self.device.fifo_config().write(|w| *w = fifo_config)?;
            any = true;
        }

        if let Some(int_ctrl) = builder.int_ctrl {
            self.device.int_ctrl().write(|w| *w = int_ctrl)?;
            any = true;
        }

        if let Some(if_conf) = builder.if_conf {
            self.device.if_conf().write(|w| *w = if_conf)?;
            any = true;
        }

        if let Some(pwr_ctrl) = builder.pwr_ctrl {
            self.device.pwr_ctrl().write(|w| *w = pwr_ctrl)?;
            any = true;
        }

        if let Some(osr) = builder.osr {
            self.device.osr().write(|w| *w = osr)?;
            any = true;
        }

        if let Some(odr) = builder.odr {
            self.device.odr().write(|w| *w = odr)?;
            any = true;
        }

        if let Some(config) = builder.config {
            self.device.config().write(|w| *w = config)?;
            any = true;
        }

        if any {
            let err_reg = self.device.err_reg().read()?;
            if err_reg.conf_err() {
                return Err(ConfigureError::InvalidConfiguration);
            }
        }

        if let Some(reference_altitude) = builder.reference_altitude {
            self.set_reference_altitude(reference_altitude);
        }

        Ok(())
    }

    /// Get the calibrated temperature.
    pub fn temperature(&mut self) -> Result<ThermodynamicTemperature, E> {
        let temperature = self.device.temperature_data().read()?;
        let coefficients = self.coefficients()?;
        Ok(coefficients.compensate_temperature(temperature.value()))
    }

    /// Get the calibrated temperature and pressure.
    pub fn temperature_and_pressure(&mut self) -> Result<(ThermodynamicTemperature, Pressure), E> {
        let data = self.device.data().read()?;
        let coefficients = self.coefficients()?;
        let temperature = coefficients.compensate_temperature(data.temperature());
        let pressure = coefficients.compensate_pressure(temperature, data.pressure());
        Ok((temperature, pressure))
    }

    /// Get the calibrated pressure.
    ///
    /// Due to how the calibration works, this function also reads the temperature.
    pub fn pressure(&mut self) -> Result<Pressure, E> {
        let (_, pressure) = self.temperature_and_pressure()?;
        Ok(pressure)
    }

    /// Get the calibrated temperature, pressure, and altitude.
    ///
    /// The altitude is calculated based on the current pressure and the reference altitude.
    /// To change the altitude, use [`Self::set_reference_altitude`].
    pub fn measure(&mut self) -> Result<Measurement, E> {
        let (temperature, pressure) = self.temperature_and_pressure()?;
        Ok(Measurement {
            temperature,
            pressure,
            altitude: crate::calculate_altitude(pressure, self.reference_altitude),
        })
    }

    /// Get the altitude based on the current pressure and the reference altitude.
    ///
    /// To change the reference altitude, use [`Self::set_reference_altitude`].
    pub fn altitude(&mut self) -> Result<Length, E> {
        let measurement = self.measure()?;
        Ok(measurement.altitude)
    }
}
