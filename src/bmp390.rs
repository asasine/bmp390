use crate::{
    Measurement,
    calibration::Coefficients,
    measurement::calculate_altitude,
    raw::Device,
    registers::{
        FifoConfig, FifoWatermark, FilterCoefficient, InterfaceConfig, InterruptControl,
        OutputDataRate, OversamplingConfig, PowerControl,
    },
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
    interrupt_control: Option<InterruptControl>,
    interface: Option<InterfaceConfig>,
    power_control: Option<PowerControl>,
    oversampling: Option<OversamplingConfig>,
    output_data_rate: Option<OutputDataRate>,
    iir_filter: Option<FilterCoefficient>,

    // Bmp390
    reference_altitude: Option<Length>,
}

impl ConfigurationBuilder {
    pub const DEFAULT: Self = Self {
        fifo_watermark: None,
        fifo_config: None,
        interrupt_control: None,
        interface: None,
        power_control: None,
        oversampling: None,
        output_data_rate: None,
        iir_filter: None,
        reference_altitude: None,
    };

    /// Create a new [`ConfigurationBuilder`] with no registers configured.
    pub const fn new() -> Self {
        Self::DEFAULT
    }

    /// Set the FIFO watermark level.
    pub fn fifo_watermark(mut self, fifo_watermark: impl Into<FifoWatermark>) -> Self {
        self.fifo_watermark = Some(fifo_watermark.into());
        self
    }

    /// Set the FIFO configuration.
    pub fn fifo_config(mut self, fifo_config: impl Into<FifoConfig>) -> Self {
        self.fifo_config = Some(fifo_config.into());
        self
    }

    /// Set the interrupt control configuration.
    pub fn interrupt_control(mut self, int_ctrl: impl Into<InterruptControl>) -> Self {
        self.interrupt_control = Some(int_ctrl.into());
        self
    }

    /// Set the serial interface settings.
    pub fn interface(mut self, if_conf: impl Into<InterfaceConfig>) -> Self {
        self.interface = Some(if_conf.into());
        self
    }

    /// Set the power control configuration.
    pub fn power_control(mut self, pwr_ctrl: impl Into<PowerControl>) -> Self {
        self.power_control = Some(pwr_ctrl.into());
        self
    }

    /// Set the oversampling settings.
    pub fn oversampling(mut self, osr: impl Into<OversamplingConfig>) -> Self {
        self.oversampling = Some(osr.into());
        self
    }

    /// Set the output data rate.
    pub fn output_data_rate(mut self, odr: impl Into<OutputDataRate>) -> Self {
        self.output_data_rate = Some(odr.into());
        self
    }

    /// Set the IIR filter coefficients.
    pub fn iir_filter(mut self, config: impl Into<FilterCoefficient>) -> Self {
        self.iir_filter = Some(config.into());
        self
    }

    /// Set the reference altitude for altitude calculations.
    ///
    /// This isn't a register itself, but is useful for updating the [`Bmp390`]
    /// during configuration.
    pub fn reference_altitude(mut self, reference_altitude: Length) -> Self {
        self.reference_altitude = Some(reference_altitude);
        self
    }
}

impl Default for ConfigurationBuilder {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigureError<E> {
    #[error(transparent)]
    Interface(#[from] E),

    #[error("The device rejected the configuration as invalid.")]
    RejectedByDevice,
}

impl<I, E> Bmp390<I>
where
    I: AsyncRegisterInterface<AddressType = u8, Error = E>,
{
    /// Get the calibration coefficients, reading them from the device if they have not been read yet.
    async fn coefficients_async(&mut self) -> Result<&Coefficients, E> {
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
                .write_async(|w| *w = fifo_watermark.into())
                .await?;
            any = true;
        }

        if let Some(fifo_config) = builder.fifo_config {
            self.device
                .fifo_config()
                .write_async(|w| *w = fifo_config.into())
                .await?;
            any = true;
        }

        if let Some(interrupt_control) = builder.interrupt_control {
            self.device
                .int_ctrl()
                .write_async(|w| *w = interrupt_control.into())
                .await?;
            any = true;
        }

        if let Some(interface) = builder.interface {
            self.device
                .if_conf()
                .write_async(|w| *w = interface.into())
                .await?;
            any = true;
        }

        if let Some(power_control) = builder.power_control {
            self.device
                .pwr_ctrl()
                .write_async(|w| *w = power_control.into())
                .await?;
            any = true;
        }

        if let Some(oversampling) = builder.oversampling {
            self.device
                .osr()
                .write_async(|w| *w = oversampling.into())
                .await?;
            any = true;
        }

        if let Some(output_data_rate) = builder.output_data_rate {
            self.device
                .odr()
                .write_async(|w| *w = output_data_rate.into())
                .await?;
            any = true;
        }

        if let Some(iir_filter) = builder.iir_filter {
            self.device
                .config()
                .write_async(|w| *w = iir_filter.into())
                .await?;
            any = true;
        }

        if any {
            let err_reg = self.device.err_reg().read_async().await?;
            if err_reg.conf_err() {
                return Err(ConfigureError::RejectedByDevice);
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
            altitude: calculate_altitude(pressure, self.reference_altitude),
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
    fn coefficients(&mut self) -> Result<&Coefficients, E> {
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
                .write(|w| *w = fifo_watermark.into())?;
            any = true;
        }

        if let Some(fifo_config) = builder.fifo_config {
            self.device
                .fifo_config()
                .write(|w| *w = fifo_config.into())?;
            any = true;
        }

        if let Some(int_ctrl) = builder.interrupt_control {
            self.device.int_ctrl().write(|w| *w = int_ctrl.into())?;
            any = true;
        }

        if let Some(if_conf) = builder.interface {
            self.device.if_conf().write(|w| *w = if_conf.into())?;
            any = true;
        }

        if let Some(pwr_ctrl) = builder.power_control {
            self.device.pwr_ctrl().write(|w| *w = pwr_ctrl.into())?;
            any = true;
        }

        if let Some(osr) = builder.oversampling {
            self.device.osr().write(|w| *w = osr.into())?;
            any = true;
        }

        if let Some(odr) = builder.output_data_rate {
            self.device.odr().write(|w| *w = odr.into())?;
            any = true;
        }

        if let Some(config) = builder.iir_filter {
            self.device.config().write(|w| *w = config.into())?;
            any = true;
        }

        if any {
            let err_reg = self.device.err_reg().read()?;
            if err_reg.conf_err() {
                return Err(ConfigureError::RejectedByDevice);
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
            altitude: calculate_altitude(pressure, self.reference_altitude),
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

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::raw;
    use core::assert_matches;
    use device_driver::FieldSet;
    use device_driver_mock::{LinearMemory, Recording, RegisterOperation, RegisterOperationRef::*};
    use std::vec;
    use uom::{
        ConstZero,
        si::{pressure::pascal, thermodynamic_temperature::degree_celsius},
    };

    const CALIBRATION_BYTES: [u8; 22] = [
        0x00, 0x98, 0x6c, 0xa9, 0x4a, 0xf9, 0xe3, 0x1c, 0x61, 0x16, 0x06, 0x01, 0x51, 0x4a, 0xde,
        0x5d, 0x03, 0xfa, 0xf9, 0x0e, 0x06, 0xf5,
    ];

    fn expected_pressure() -> Pressure {
        Pressure::new::<pascal>(98370.55)
    }

    fn expected_temperature() -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(25.770_746)
    }

    fn expected_altitude() -> Length {
        Length::new::<meter>(248.78754)
    }

    /// Create a [`Recording`] interface with the pressure, temperature, and calibration data preloaded.
    fn interface() -> Recording<LinearMemory> {
        let interface = LinearMemory::default();
        let mut d = Device::new(interface);
        let Ok(()) = d.pressure_data().write(|w| w.set_value(0x6b_b3_cb));
        let Ok(()) = d.temperature_data().write(|w| w.set_value(0x82_ba_d1));
        let Ok(()) = d.calibration_data().write(|w| {
            w.get_inner_buffer_mut().copy_from_slice(&CALIBRATION_BYTES);
        });

        Recording {
            interface: d.interface,
            operations: vec![],
        }
    }

    #[tokio::test]
    async fn reads_temperature_and_compensates() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(temperature) = bmp390.temperature_async().await;
        assert_eq!(temperature, expected_temperature());
        assert_matches!(
            bmp390.device.interface.ops_as_ref().as_slice(),
            &[
                Read {
                    address: 0x07,
                    data: temperature_data,
                    ..
                },
                Read {
                    address: 0x30,
                    data: calibration_data,
                    ..
                },
            ]
            if temperature_data == &[0; 3] && calibration_data == &[0; 22]
        );
    }

    #[tokio::test]
    async fn reads_pressure() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(pressure) = bmp390.pressure_async().await;
        assert_eq!(pressure, expected_pressure());
    }

    #[tokio::test]
    async fn reads_temperature_and_pressure() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(measurement) = bmp390.temperature_and_pressure_async().await;
        assert_eq!(measurement.0, expected_temperature());
        assert_eq!(measurement.1, expected_pressure());
    }

    #[tokio::test]
    async fn reads_altitude() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(altitude) = bmp390.altitude_async().await;
        assert_eq!(altitude, expected_altitude());
    }

    #[tokio::test]
    async fn measure_reads_temperature_pressure_and_altitude() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(measurement) = bmp390.measure_async().await;
        assert_eq!(measurement.temperature, expected_temperature());
        assert_eq!(measurement.pressure, expected_pressure());
        assert_eq!(measurement.altitude, expected_altitude());
    }

    #[tokio::test]
    async fn altitude_uses_custom_reference() {
        let mut bmp390 = Bmp390::new(interface());
        bmp390.set_reference_altitude(expected_altitude());
        let Ok(altitude) = bmp390.altitude_async().await;
        assert_eq!(altitude, Length::ZERO);
    }

    #[tokio::test]
    async fn caches_calibration_coefficients() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(_) = bmp390.temperature_async().await;
        let Ok(_) = bmp390.temperature_async().await;
        let calibration_reads = bmp390
            .device
            .interface
            .operations
            .iter()
            .filter(|operation| matches!(operation, RegisterOperation::Read { address: 0x30, .. }))
            .count();

        assert_eq!(calibration_reads, 1);
    }

    #[tokio::test]
    async fn configure_writes_register_and_checks_for_errors() {
        let mut bmp390 = Bmp390::new(interface());
        let osr = OversamplingConfig::try_from(raw::field_sets::Osr::from([0x12])).unwrap();
        bmp390
            .configure_async(ConfigurationBuilder::new().oversampling(osr))
            .await
            .unwrap();

        assert_matches!(
            bmp390.device.interface.ops_as_ref().as_slice(),
            &[
                Write {
                    address: 0x1c,
                    data: &[0x12],
                    ..
                },
                Read {
                    address: 0x02,
                    data: &[0],
                    ..
                },
            ]
        );
    }

    #[tokio::test]
    async fn configure_reports_invalid_configuration() {
        let interface = interface();
        let mut bmp390 = Bmp390::new(interface);

        // make the device report a configuration error
        bmp390
            .device()
            .err_reg()
            .write(|w| w.set_conf_err(true))
            .unwrap();

        let osr = OversamplingConfig::try_from(raw::field_sets::Osr::from([0x12])).unwrap();
        let result = bmp390
            .configure_async(ConfigurationBuilder::new().oversampling(osr))
            .await;

        assert_eq!(result, Err(ConfigureError::RejectedByDevice));
    }

    #[test]
    fn synchronous_api_uses_register_interface() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(measurement) = bmp390.measure();
        assert_eq!(measurement.temperature, expected_temperature());
        assert_eq!(measurement.pressure, expected_pressure());
        assert_eq!(measurement.altitude, expected_altitude());
    }
}
