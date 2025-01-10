//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The
//! sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt
//! functionality, lower power consumption, and a new FIFO functionality. The integrated 512 byte FIFO buffer supports
//! low power applications and prevents data loss in non-real-time systems.
//!
//! [`Bmp390`] is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the
//! sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal_async::i2c`]
//! traits to be compatible with a wide range of embedded platforms. Measurements utilize the [`uom`] crate to provide
//! automatic, type-safe, and zero-cost units of measurement for [`Measurement`].
//!
//! # Example
//! ```no_run
//! # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
//! # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
//! use bmp390::Bmp390;
//! let config = bmp390::Configuration::default();
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
//! let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
//! let measurement = sensor.measure().await?;
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
//! # Synchronous API
//! The synchronous API is available behind the `sync` feature flag. It's driver is [`sync::Bmp390`] and functions
//! similarly to the asynchronous driver, but with synchronous methods.
//!
//! By default, the synchronous API is disabled.

#![no_std]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

use defmt::{debug, trace, Format};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use libm::powf;
use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
use uom::si::length::{foot, meter};
use uom::si::pressure::{hectopascal, pascal};
use uom::si::thermodynamic_temperature::degree_celsius;

mod registers;

#[cfg(feature = "sync")]
pub mod sync;

pub use registers::*;

/// Errors that can occur when communicating with the BMP390 barometer.
#[derive(Debug, Clone, Copy, Format)]
pub enum Error<E> {
    /// An error occurred while communicating with the BMP390 over I2C. The inner error contains the specific error.
    I2c(E),

    /// The BMP390's chip ID did not match the expected value of `0x60`. The actual chip ID is provided.
    WrongChip(u8),

    /// A fatal error occurred on the BMP390. See [`ErrReg`] for more.
    Fatal,

    /// A command error occurred on the BMP390. See [`ErrReg`] for more.
    Command,

    /// A configuration error occurred on the BMP390. See [`ErrReg`] for more.
    Configuration,
}

/// Note: [`embedded_hal_async::i2c::ErrorKind`] is an alias for [`embedded_hal::i2c::ErrorKind`], so the one impl
/// covers both.
impl From<embedded_hal_async::i2c::ErrorKind> for Error<embedded_hal_async::i2c::ErrorKind> {
    fn from(error: embedded_hal_async::i2c::ErrorKind) -> Self {
        Error::I2c(error)
    }
}

/// A single measurement from the [`Bmp390`] barometer.
///
/// Measurements utilize the [`uom`] crate to provide automatic, type-safe, and zero-cost units of measurement.
///
/// # Example
/// ```
/// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
/// # use uom::si::pressure::pascal;
/// # use uom::si::length::meter;
/// # use uom::si::thermodynamic_temperature::degree_celsius;
/// let measurement = bmp390::Measurement {
///    pressure: Pressure::new::<pascal>(90_240.81),
///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
///    altitude: Length::new::<meter>(1000.0),
/// };
///
/// defmt::info!("Measurement: {}", measurement);
/// ```
///
/// Note: these examples show creation of [`Measurement`] structs directly. In practice you would receive these from
/// [`Bmp390::measure`].
///
/// Conversion between units is easy with the [`uom`] crate. For example, to convert to imperial units:
/// ```
/// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
/// # use uom::si::pressure::pascal;
/// # use uom::si::length::meter;
/// # use uom::si::thermodynamic_temperature::degree_celsius;
/// # let measurement = bmp390::Measurement {
/// #    pressure: Pressure::new::<pascal>(90_240.81),
/// #    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
/// #    altitude: Length::new::<meter>(1000.0),
/// # };
/// use uom::si::pressure::millimeter_of_mercury;
/// use uom::si::thermodynamic_temperature::degree_fahrenheit;
/// use uom::si::length::foot;
///
/// // "Pressure: 676.9753 mmHg, Temperature: 77 °F, Altitude: 3280.84 feet"
/// defmt::info!("Pressure: {} mmHg, temperature: {} °F, altitude: {} feet",
///     measurement.pressure.get::<millimeter_of_mercury>(),
///     measurement.temperature.get::<degree_fahrenheit>(),
///     measurement.altitude.get::<foot>());
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// The pressure as a [`Pressure`], allowing for easy conversion to any unit of pressure.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::pressure::millimeter_of_mercury;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Pressure: 676.9753 mmHg"
    /// defmt::info!("Pressure: {} mmHg", measurement.pressure.get::<millimeter_of_mercury>());
    /// ```
    pub pressure: Pressure,

    /// The temperature as a [`ThermodynamicTemperature`], allowing for easy conversion to any unit of temperature.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::thermodynamic_temperature::degree_fahrenheit;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Temperature: 77 °F"
    /// defmt::info!("Temperature: {} °F", measurement.temperature.get::<degree_fahrenheit>());
    /// ```
    pub temperature: ThermodynamicTemperature,

    /// The altitude as a [`Length`], allowing for easy conversion to any unit of length.
    ///
    /// # Example
    /// ```
    /// # use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};
    /// # use uom::si::pressure::pascal;
    /// # use uom::si::length::meter;
    /// # use uom::si::thermodynamic_temperature::degree_celsius;
    /// use uom::si::length::foot;
    /// let measurement = bmp390::Measurement {
    ///    pressure: Pressure::new::<pascal>(90_240.81),
    ///    temperature: ThermodynamicTemperature::new::<degree_celsius>(25.0),
    ///    altitude: Length::new::<meter>(1000.0),
    /// };
    ///
    /// // "Length: 3280.84 feet"
    /// defmt::info!("Length: {} feet", measurement.altitude.get::<foot>());
    /// ```
    pub altitude: Length,
}

impl Format for Measurement {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Pressure: {} Pa, Temperature: {} °C, Altitude: {} m",
            self.pressure.get::<pascal>(),
            self.temperature.get::<degree_celsius>(),
            self.altitude.get::<meter>()
        );
    }
}

impl core::fmt::Display for Measurement {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "Pressure: {} Pa, Temperature: {} °C, Altitude: {} m",
            self.pressure.get::<pascal>(),
            self.temperature.get::<degree_celsius>(),
            self.altitude.get::<meter>(),
        )
    }
}

/// The BMP390 barometer's I2C addresses, either `0x76` or `0x77`.
///
///  The BMP390 can be configured to use two different addresses by either pulling the `SDO` pin down to `GND`
/// (`0x76` via [`Address::Down`]) or up to `V_DDIO` (`0x77` via [`Address::Up`]).
#[derive(Debug, Clone, Copy, Format)]
pub enum Address {
    /// `0x76`: The BMP390's address when `SDO` is pulled up to `GND`.
    Down = 0x76,

    /// `0x77`: The BMP390's address when `SDO` is pulled down to `V_DDIO`
    Up = 0x77,
}

impl From<Address> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(address: Address) -> u8 {
        address as u8
    }
}

/// Output from the BMP390 consists of ADC outputs.
///
/// These must be compensated using formulas from the datasheet to obtain the actual temperature and pressure values,
/// using coefficients stored in non-volatile memory (NVM).
///
/// # Datasheet
/// - Section 3.11 Output compensation.
/// - Appendix A: Computation formulae reference implementation.
#[derive(Debug, Clone, Copy, Format)]
struct CalibrationCoefficients {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
}

impl CalibrationCoefficients {
    /// Read the calibration coefficients from the BMP390's NVM registers and convert them to into a set of
    /// floating-point calibration coefficients for the formulas implemented in the compensation functions.
    async fn try_from_i2c<I: I2c>(address: Address, i2c: &mut I) -> Result<Self, Error<I::Error>> {
        let mut calibration_coefficient_regs = [0; 21];
        i2c.write_read(
            address.into(),
            &Self::write_read_write_transaction(),
            &mut calibration_coefficient_regs,
        )
        .await
        .map_err(Error::I2c)?;

        Ok(Self::from_registers(&calibration_coefficient_regs))
    }

    /// Calculate the calibration coefficients from the raw register data in registers [`Register::NVM_PAR_T1_0`] to
    /// [`Register::NVM_PAR_P11`].
    ///
    /// # Datasheet
    /// Apendix A, Section 8.4
    fn from_registers(data: &[u8; 21]) -> Self {
        trace!("NVM_PAR: {=[u8]:#04x}", *data);
        let nvm_par_t1: u16 = (data[1] as u16) << 8 | data[0] as u16;
        let nvm_par_t2: u16 = (data[3] as u16) << 8 | data[2] as u16;
        let nvm_par_t3: i8 = data[4] as i8;
        let nvm_par_p1: i16 = (data[6] as i16) << 8 | data[5] as i16;
        let nvm_par_p2: i16 = (data[8] as i16) << 8 | data[7] as i16;
        let nvm_par_p3: i8 = data[9] as i8;
        let nvm_par_p4: i8 = data[10] as i8;
        let nvm_par_p5: u16 = (data[12] as u16) << 8 | data[11] as u16;
        let nvm_par_p6: u16 = (data[14] as u16) << 8 | data[13] as u16;
        let nvm_par_p7: i8 = data[15] as i8;
        let nvm_par_p8: i8 = data[16] as i8;
        let nvm_par_p9: i16 = (data[18] as i16) << 8 | data[17] as i16;
        let nvm_par_p10: i8 = data[19] as i8;
        let nvm_par_p11: i8 = data[20] as i8;

        Self {
            par_t1: (nvm_par_t1 as f32) / 0.003_906_25,    // 2^-8
            par_t2: (nvm_par_t2 as f32) / 1_073_741_824.0, // 2^30
            par_t3: (nvm_par_t3 as f32) / 281_474_976_710_656.0, // 2^48
            par_p1: ((nvm_par_p1 as f32) - 16_384.0) / 1_048_576.0, // 2^14 / 2^20
            par_p2: ((nvm_par_p2 as f32) - 16_384.0) / 536_870_912.0, // 2^14 / 2^29
            par_p3: (nvm_par_p3 as f32) / 4_294_967_296.0, // 2^32
            par_p4: (nvm_par_p4 as f32) / 137_438_953_472.0, // 2^37
            par_p5: (nvm_par_p5 as f32) / 0.125,           // 2^-3
            par_p6: (nvm_par_p6 as f32) / 64.0,            // 2^6
            par_p7: (nvm_par_p7 as f32) / 256.0,           // 2^8
            par_p8: (nvm_par_p8 as f32) / 32768.0,         // 2^15
            par_p9: (nvm_par_p9 as f32) / 281_474_976_710_656.0, //2^48
            par_p10: (nvm_par_p10 as f32) / 281_474_976_710_656.0, // 2^48
            par_p11: (nvm_par_p11 as f32) / 36_893_488_147_419_103_232.0, // 2^65
        }
    }

    /// Compensate a temperature reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.5
    fn compensate_temperature(&self, temperature_uncompensated: u32) -> ThermodynamicTemperature {
        // This could be done in fewer expressions, but it's broken down for clarity and to match the datasheet
        let uncompensated = temperature_uncompensated as f32;
        let partial_data1 = uncompensated - self.par_t1;
        let partial_data2 = partial_data1 * self.par_t2;
        let temperature = partial_data2 + (partial_data1 * partial_data1) * self.par_t3;
        ThermodynamicTemperature::new::<degree_celsius>(temperature)
    }

    /// Compensate a pressure reading according to calibration coefficients.
    ///
    /// # Datasheet
    /// Apendix A, Section 8.6
    fn compensate_pressure(
        &self,
        temperature: ThermodynamicTemperature,
        pressure_uncompensated: u32,
    ) -> Pressure {
        // This could be done in fewer expressions, but it's broken down for clarity and to match the datasheet
        let uncompensated = pressure_uncompensated as f32;
        let temperature = temperature.get::<degree_celsius>();
        let partial_data1 = self.par_p6 * temperature;
        let partial_data2 = self.par_p7 * temperature * temperature;
        let partial_data3 = self.par_p8 * temperature * temperature * temperature;
        let partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.par_p2 * temperature;
        let partial_data2 = self.par_p3 * temperature * temperature;
        let partial_data3 = self.par_p4 * temperature * temperature * temperature;
        let partial_out2 =
            uncompensated * (self.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = uncompensated * uncompensated;
        let partial_data2 = self.par_p9 + self.par_p10 * temperature;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 =
            partial_data3 + uncompensated * uncompensated * uncompensated * self.par_p11;

        let pressure = partial_out1 + partial_out2 + partial_data4;
        Pressure::new::<pascal>(pressure)
    }

    /// Gets the bytes to write in a write-read transaction to the BMP390 to read the calibration coefficients. This
    /// must be combined with a 21-byte read in a combined write-read burst.
    fn write_read_write_transaction() -> [u8; 1] {
        [Register::NVM_PAR_T1_0.into()]
    }
}

/// Configuration for the BMP390 barometer.
#[derive(Debug, Clone, Copy, Format)]
pub struct Configuration {
    /// Enabling and disabling the pressure and temperature measurements and the power mode.
    pub power_control: PowerControl,

    /// The oversampling settings for pressure and temperature measurements.
    pub oversampling: Osr,

    /// The output data rate settings.
    pub output_data_rate: Odr,

    /// IIR filter coefficient settings.
    pub iir_filter: Config,
}

impl Default for Configuration {
    /// Default configuration for the BMP390 barometer. This configuration enables pressure and temperature measurement
    /// with normal power mode, x8 oversampling for pressure and x1 oversampling for temperature, an output data rate of
    /// 50 Hz, and a IIR filter coefficient of 4. This corresponds to a "standard resolution" configuration as
    /// recommended by the datasheet Section 3.5. Filter selection.
    fn default() -> Self {
        Self {
            power_control: PowerControl {
                enable_pressure: true,
                enable_temperature: true,
                mode: PowerMode::Normal,
            },
            oversampling: Osr {
                pressure: Oversampling::X8,
                temperature: Oversampling::X1,
            },
            output_data_rate: Odr {
                odr_sel: OdrSel::ODR_50,
            },
            iir_filter: Config {
                iir_filter: IirFilter::coef_15,
            },
        }
    }
}

impl Configuration {
    /// Convert the configuration to a byte array that can be written to the BMP390's registers.
    /// The byte array contains both the register address and the register value.
    pub fn to_write_bytes(&self) -> [u8; 8] {
        [
            Register::PWR_CTRL.into(),
            self.power_control.into(),
            Register::OSR.into(),
            self.oversampling.into(),
            Register::ODR.into(),
            self.output_data_rate.into(),
            Register::CONFIG.into(),
            self.iir_filter.into(),
        ]
    }
}

/// A driver for the BMP390 pressure sensor over any [`I2c`] implementation.
///
/// This driver utilizes [`uom`] to provide automatic, type-safe, and zero-cost units of measurement. Measurements can
/// be retrieved with [`Bmp390::measure`], which returns a [`Measurement`] struct containing the pressure, temperature,
/// and altitude. The altitude is calculated based on the current pressure, standard atmospheric pressure at sea level,
/// and a reference altitude, which can be set with [`Bmp390::set_reference_altitude`]. The reference altitude defaults
/// to zero, so the default altitude is measured from sea level.
///
/// # Example
/// ```no_run
/// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
/// use bmp390::Bmp390;
/// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
/// let config = bmp390::Configuration::default();
/// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
/// let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
/// let measurement = sensor.measure().await?;
/// defmt::info!("Measurement: {}", measurement);
/// # Ok(())
/// # }
/// ```
pub struct Bmp390<I> {
    /// The I2C bus the barometer is connected to.
    i2c: I,

    /// The I2C address of the barometer.
    address: Address,

    /// The calibration coefficients for the barometer to compensate temperature and pressure measurements.
    coefficients: CalibrationCoefficients,

    /// The reference altitude for altitude calculations.
    ///
    /// By default, this is zero. set to the standard atmospheric pressure at sea level, 1013.25 hPa. It can be set to
    /// a different value using [`Bmp390::set_reference_altitude`] to calculate the altitude relative to a different
    /// reference point.
    altitude_reference: Length,
}

impl<I, E> Bmp390<I>
where
    I: I2c<Error = E>,
{
    /// Creates a new BMP390 driver. This will initialize the barometer with the provided configuration.
    /// It will additionally delay for 2 ms to allow the barometer to start up and read the calibration coefficients
    /// for temperature and pressure measuring.
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// use bmp390::Bmp390;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let measurement = sensor.measure().await?;
    /// defmt::info!("Measurement: {}", measurement);
    /// # Ok(())
    /// # }
    /// ```
    pub async fn try_new<D: DelayNs>(
        mut i2c: I,
        address: Address,
        mut delay: D,
        config: &Configuration,
    ) -> Result<Self, Error<E>> {
        // 2 ms time to first communication (Datsheet Section 1, Table 2)
        delay.delay_ms(2).await;

        let mut data = [0; 2];
        i2c.write_read(address.into(), &[Register::CHIP_ID.into()], &mut data)
            .await
            .map_err(Error::I2c)?;

        let chip_id = data[0];
        let rev_id = data[1];

        debug!("CHIP_ID = {=u8:#04x}; REV_ID = {=u8:#04x}", chip_id, rev_id);
        if chip_id != 0x60 {
            return Err(Error::WrongChip(chip_id));
        }

        // read Register::EVENT and INT_STATUS in a burst read to clear the event and interrupt status flags
        let mut data = [0; 2];
        i2c.write_read(address.into(), &[Register::EVENT.into()], &mut data)
            .await
            .map_err(Error::I2c)?;

        // write configuration after clearing interrupt status flags so that they are accurate from here on
        i2c.write(address.into(), &config.to_write_bytes())
            .await
            .map_err(Error::I2c)?;

        // read Register::ERR_REG after writing config to determine if configuration was successful and to clear the error status flags
        let mut err_reg = [0; 1];
        i2c.write_read(address.into(), &[Register::ERR_REG.into()], &mut err_reg)
            .await
            .map_err(Error::I2c)
            .and_then(move |_| {
                let err_reg = ErrReg::from(err_reg[0]);
                if err_reg.fatal_err {
                    Err(Error::<E>::Fatal)
                } else if err_reg.cmd_err {
                    Err(Error::<E>::Command)
                } else if err_reg.conf_err {
                    Err(Error::<E>::Configuration)
                } else {
                    Ok(())
                }
            })?;

        let coefficients = CalibrationCoefficients::try_from_i2c(address, &mut i2c).await?;

        Ok(Self::new_with_coefficients(i2c, address, coefficients))
    }

    /// Creates a new BMP390 driver with known calibration coefficients.
    fn new_with_coefficients(
        i2c: I,
        address: Address,
        coefficients: CalibrationCoefficients,
    ) -> Self {
        Self {
            i2c,
            address,
            coefficients,
            altitude_reference: Length::new::<meter>(0.0),
        }
    }

    /// Reads the temperature from the barometer as a [`ThermodynamicTemperature`].
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::Bmp390;
    /// use uom::si::thermodynamic_temperature::degree_celsius;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let temperature = sensor.temperature().await?;
    /// defmt::info!("Temperature: {} °C", temperature.get::<degree_celsius>());
    /// # Ok(())
    /// # }
    /// ```
    pub async fn temperature(&mut self) -> Result<ThermodynamicTemperature, Error<E>> {
        // Burst read: only address DATA_3 (temperature XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_3.into()];
        let mut read = [0; 3];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
            .await
            .map_err(Error::I2c)?;

        // DATA_3 is the LSB, DATA_5 is the MSB
        let temperature = u32::from(read[0]) | u32::from(read[1]) << 8 | u32::from(read[2]) << 16;
        let temperature = self.coefficients.compensate_temperature(temperature);
        Ok(temperature)
    }

    /// Reads the pressure from the barometer as a [`Pressure`].
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::Bmp390;
    /// use uom::si::pressure::hectopascal;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let pressure = sensor.pressure().await?;
    /// defmt::info!("Pressure: {} hPa", pressure.get::<hectopascal>());
    /// # Ok(())
    /// # }
    /// ```
    pub async fn pressure(&mut self) -> Result<Pressure, Error<E>> {
        // pressure requires temperature to compensate, so just measure both
        let measurement = self.measure().await?;
        Ok(measurement.pressure)
    }

    /// Measures the pressure and temperature from the barometer.
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::Bmp390;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let measurement = sensor.measure().await?;
    /// defmt::info!("Measurement: {}", measurement);
    /// # Ok(())
    /// # }
    /// ```
    pub async fn measure(&mut self) -> Result<Measurement, Error<E>> {
        // Burst read: only address DATA_0 (pressure XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_0.into()];
        let mut read = [0; 6];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
            .await
            .map_err(Error::I2c)?;

        trace!("DATA = {=[u8]:#04x}", read);

        // pressure is 0:2 (XLSB, LSB, MSB), temperature is 3:5 (XLSB, LSB, MSB)
        let temperature = u32::from(read[3]) | u32::from(read[4]) << 8 | u32::from(read[5]) << 16;
        let temperature = self.coefficients.compensate_temperature(temperature);

        let pressure = u32::from(read[0]) | u32::from(read[1]) << 8 | u32::from(read[2]) << 16;
        let pressure = self.coefficients.compensate_pressure(temperature, pressure);

        Ok(Measurement {
            temperature,
            pressure,
            altitude: calculate_altitude(pressure, self.altitude_reference),
        })
    }

    /// Set the reference altitude for altitude calculations.
    ///
    /// Following this, the altitude can be calculated using [`Bmp390::altitude`]. If the current pressure matches
    /// the pressure when the reference altitude is set, the altitude will be 0.
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::Bmp390;
    /// # use uom::si::length::meter;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let initial_altitude = sensor.altitude().await?;
    /// sensor.set_reference_altitude(initial_altitude);
    ///
    /// // Some time later...
    /// let altitude = sensor.altitude().await?;
    /// defmt::info!("Altitude: {} meters", altitude.get::<meter>());
    /// # Ok(())
    /// # }
    /// ```
    pub fn set_reference_altitude(&mut self, altitude: Length) {
        self.altitude_reference = altitude;
    }

    /// Calculates the latest altitude measurement as a [`Length`] after retrieving the latest pressure measurement.
    ///
    /// The altitude is calculating following the [NOAA formula](https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf).
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::Bmp390;
    /// use uom::si::length::foot;
    /// # async fn run() -> Result<(), bmp390::Error<embedded_hal_async::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
    /// let altitude = sensor.altitude().await?;
    /// defmt::info!("Length: {} feet", altitude.get::<foot>());
    /// # Ok(())
    /// # }
    /// ```
    pub async fn altitude(&mut self) -> Result<Length, Error<E>> {
        let pressure = self.pressure().await?;
        Ok(calculate_altitude(pressure, self.altitude_reference))
    }
}

/// Calculate the altitude based on the pressure, sea level pressure, and the reference altitude.
///
/// The altitude is calculating following the [NOAA formula](https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf).
fn calculate_altitude(pressure: Pressure, altitude_reference: Length) -> Length {
    let sea_level = Pressure::new::<hectopascal>(1013.25);
    let above_sea_level =
        Length::new::<foot>(145366.45 * (1.0 - powf((pressure / sea_level).value, 0.190284)));

    above_sea_level - altitude_reference
}

#[cfg(test)]
mod tests {
    extern crate std;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, NoopDelay, Transaction as DelayTransaction};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as I2cTransaction};
    use std::prelude::rust_2021::*;
    use std::vec;
    use uom::ConstZero;

    use super::*;

    /// Bytes for the DATA registers (0x04 .. 0x09) for a pressure and temperature measurement.
    const PRESSURE_TEMPERATURE_BYTES: [u8; 6] = [0xcb, 0xb3, 0x6b, 0xd1, 0xba, 0x82];

    /// The [`Measurement::pressure`] value for [`PRESSURE_TEMPERATURE_BYTES`] when compensated by [`CalibrationCoefficients::default()`].
    fn expected_pressure() -> Pressure {
        Pressure::new::<pascal>(98370.55)
    }

    /// Bytes for the DATA registers (0x07 .. 0x09) for a temperature measurement.
    const TEMPERATURE_BYTES: [u8; 3] = [0xd1, 0xba, 0x82];

    /// The [`Measurement::temperature`] value for [`TEMPERATURE_BYTES`] when compensated by [`CalibrationCoefficients::default()`].
    fn expected_temperature() -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(25.770_746)
    }

    /// The [`Measurement::altitude`] value for [`expected_pressure()`] and a reference pressure of 1013.25 hPa.
    fn expected_altitude() -> Length {
        Length::new::<meter>(248.78754)
    }

    impl Default for CalibrationCoefficients {
        fn default() -> Self {
            // NVM_PAR registers (0x31 .. 0x45) from a real BMP390, rev 0x01
            Self::from_registers(&[
                0x98, 0x6c, 0xa9, 0x4a, 0xf9, 0xe3, 0x1c, 0x61, 0x16, 0x06, 0x01, 0x51, 0x4a, 0xde,
                0x5d, 0x03, 0xfa, 0xf9, 0x0e, 0x06, 0xf5,
            ])
        }
    }

    fn get_try_new_transactions(
        addr: Address,
        configuration: &Configuration,
        err_reg: &ErrReg,
        event: &Event,
        int_status: &IntStatus,
    ) -> [I2cTransaction; 5] {
        [
            // CHIP_ID is read in a 2-byte burst to also read REV_ID
            I2cTransaction::write_read(
                addr.into(),
                vec![Register::CHIP_ID.into()],
                vec![0x60, 0x01],
            ),
            // EVENT and INT_STATUS are read in a 2-byte burst
            I2cTransaction::write_read(
                addr.into(),
                vec![Register::EVENT.into()],
                vec![u8::from(*event), u8::from(*int_status)],
            ),
            I2cTransaction::write(addr.into(), configuration.to_write_bytes().to_vec()),
            I2cTransaction::write_read(
                addr.into(),
                vec![Register::ERR_REG.into()],
                vec![u8::from(*err_reg)],
            ),
            I2cTransaction::write_read(
                addr.into(),
                CalibrationCoefficients::write_read_write_transaction().to_vec(),
                vec![0; 21],
            ),
        ]
    }

    #[tokio::test]
    async fn test_try_new() {
        // Several things are implicitly tested here:
        // 1. The chip ID is read and checked => Ok
        // 2. The rev ID is read in the same burst as chip ID
        // 3. The event and int status registers are read in a burst to clear them
        // 4. The configuration is written
        // 5. The ERR_REG is read to check for errors
        // 6. The calibration coefficients are read

        let addr = Address::Up;
        let config = Configuration::default();
        let expectations = get_try_new_transactions(addr, &config, &0.into(), &0.into(), &0.into());
        let mut i2c = Mock::new(&expectations);
        let mut delay = CheckedDelay::new(&[
            DelayTransaction::async_delay_ms(2), // time to first communication
        ]);

        let _bmp390 = Bmp390::try_new(i2c.clone(), addr, delay.clone(), &config)
            .await
            .unwrap();

        delay.done();
        i2c.done();
    }

    #[tokio::test]
    async fn test_reads_temperature_and_compensates() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_3.into()],
            TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let temperature = bmp390.temperature().await.unwrap();
        assert_eq!(temperature, expected_temperature());
        i2c.done();
    }

    #[tokio::test]
    async fn test_reads_pressure() {
        let addr = Address::Up;

        // NOTE: a pressure read requires a temperature read, so response is 6 bytes
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            PRESSURE_TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let pressure = bmp390.pressure().await.unwrap();
        assert_eq!(pressure, expected_pressure());
        i2c.done();
    }

    #[tokio::test]
    async fn test_measure_reads_temperature_and_pressure() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            PRESSURE_TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let measurement = bmp390.measure().await.unwrap();
        assert_eq!(measurement.temperature, expected_temperature());
        assert_eq!(measurement.pressure, expected_pressure());
        i2c.done();
    }

    #[tokio::test]
    async fn test_altitude() {
        let addr = Address::Up;

        // NOTE: a pressure read requires a temperature read, so response is 6 bytes
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            PRESSURE_TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let altitude = bmp390.altitude().await.unwrap();
        assert_eq!(altitude, expected_altitude());
        i2c.done();
    }

    #[tokio::test]
    async fn test_altitude_custom_reference() {
        let addr = Address::Up;

        // NOTE: a pressure read requires a temperature read, so response is 6 bytes
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            PRESSURE_TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());

        bmp390.set_reference_altitude(expected_altitude());
        let altitude = bmp390.altitude().await.unwrap();
        assert_eq!(altitude, Length::ZERO);
        i2c.done();
    }

    #[tokio::test]
    async fn test_chip_id_incorrect() {
        let addr = Address::Up;

        let mut expectations = get_try_new_transactions(
            addr,
            &Configuration::default(),
            &0.into(),
            &0.into(),
            &0.into(),
        )
        .into_iter()
        .take(1)
        .collect::<Vec<_>>();

        expectations[0] = I2cTransaction::write_read(
            addr.into(),
            vec![Register::CHIP_ID.into()],
            vec![0x42, 0x01],
        );

        let mut i2c = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default()).await;
        assert!(matches!(result, Err(Error::WrongChip(0x42))));
        i2c.done();
    }

    #[tokio::test]
    async fn test_fatal_error() {
        let addr = Address::Up;

        let fatal_err = ErrReg {
            fatal_err: true,
            cmd_err: false,
            conf_err: false,
        };

        let expectations = get_try_new_transactions(
            addr,
            &Configuration::default(),
            &fatal_err.into(),
            &0.into(),
            &0.into(),
        )
        .into_iter()
        .take(4)
        .collect::<Vec<_>>();

        let mut i2c = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default()).await;
        assert!(matches!(result, Err(Error::Fatal)));
        // assert_matches!(result, Err(Error::Fatal))); // TODO: use assert_matches once it's stable
        i2c.done();
    }

    #[tokio::test]
    async fn test_command_error() {
        let addr = Address::Up;

        let cmd_err = ErrReg {
            fatal_err: false,
            cmd_err: true,
            conf_err: false,
        };

        let expectations = get_try_new_transactions(
            addr,
            &Configuration::default(),
            &cmd_err.into(),
            &0.into(),
            &0.into(),
        )
        .into_iter()
        .take(4)
        .collect::<Vec<_>>();

        let mut i2c = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default()).await;
        assert!(matches!(result, Err(Error::Command)));
        i2c.done();
    }

    #[tokio::test]
    async fn test_configuration_error() {
        let addr = Address::Up;

        let conf_err = ErrReg {
            fatal_err: false,
            cmd_err: false,
            conf_err: true,
        };

        let expectations = get_try_new_transactions(
            addr,
            &Configuration::default(),
            &conf_err.into(),
            &0.into(),
            &0.into(),
        )
        .into_iter()
        .take(4)
        .collect::<Vec<_>>();

        let mut i2c = Mock::new(&expectations);
        let delay = NoopDelay::new();
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default()).await;
        assert!(matches!(result, Err(Error::Configuration)));
        i2c.done();
    }

    #[tokio::test]
    async fn test_any_other_error() {
        // Test that the driver handles unexpected bits in the ERR_REG register gracefully (i.e. doesn't panic or error)
        let addr = Address::Up;

        for err_reg_bits in 0..=7 {
            let err_reg = ErrReg::from(err_reg_bits);
            if err_reg.fatal_err || err_reg.cmd_err || err_reg.conf_err {
                // skip the error flags we've already tested, we're looking for how the driver handles unexpected bits in this register
                continue;
            }

            let mut expectations = get_try_new_transactions(
                addr,
                &Configuration::default(),
                &0.into(),
                &0.into(),
                &0.into(),
            );

            expectations[3] = I2cTransaction::write_read(
                addr.into(),
                vec![Register::ERR_REG.into()],
                vec![err_reg_bits],
            );

            let mut i2c = Mock::new(&expectations);
            let delay = NoopDelay::new();
            let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default()).await;
            assert!(
                result.is_ok(),
                "Unexpected error with ERR_REG = {:#010b}",
                err_reg_bits
            );

            i2c.done();
        }
    }
}
