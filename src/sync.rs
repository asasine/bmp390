//! This module provides a synchronous verison of the [`Bmp390`] driver, built on top of [`embedded_hal::i2c`].
//!
//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The
//! sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt
//! functionality, lower power consumption, and a new FIFO functionality. The integrated 512 byte FIFO buffer supports
//! low power applications and prevents data loss in non-real-time systems.
//!
//! [`Bmp390`] is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the
//! sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal::i2c`]
//! traits to be compatible with a wide range of embedded platforms. Measurements utilize the [`uom`] crate to provide
//! automatic, type-safe, and zero-cost units of measurement for [`Measurement`].
//!
//! # Example
//! ```no_run
//! # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
//! # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
//! use bmp390::sync::Bmp390;
//! let config = bmp390::Configuration::default();
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
//! let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
//! let measurement = sensor.measure()?;
//! defmt::info!("Measurement: {}", measurement);
//! # Ok(())
//! # }
//! ```

use crate::{
    calculate_altitude, registers::*, Address, CalibrationCoefficients, Configuration, Error,
    Measurement,
};

use defmt::{debug, trace};
use embedded_hal::{delay::DelayNs, i2c::I2c};
use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};

impl CalibrationCoefficients {
    /// Read the calibration coefficients from the BMP390's NVM registers and convert them to into a set of
    /// floating-point calibration coefficients for the formulas implemented in the compensation functions.
    fn try_from_i2c_sync<I: embedded_hal::i2c::I2c>(
        address: Address,
        i2c: &mut I,
    ) -> Result<Self, Error<I::Error>> {
        let mut calibration_coefficient_regs = [0; 21];
        i2c.write_read(
            address.into(),
            &Self::write_read_write_transaction(),
            &mut calibration_coefficient_regs,
        )
        .map_err(Error::I2c)?;

        Ok(Self::from_registers(&calibration_coefficient_regs))
    }
}

/// A driver for the BMP390 pressure sensor over any synchronous [`I2c`] implementation.
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
/// use bmp390::sync::Bmp390;
/// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
/// let config = bmp390::Configuration::default();
/// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
/// let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
/// let measurement = sensor.measure()?;
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
    /// use bmp390::sync::Bmp390;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let measurement = sensor.measure()?;
    /// defmt::info!("Measurement: {}", measurement);
    /// # Ok(())
    /// # }
    /// ```
    pub fn try_new<D: DelayNs>(
        mut i2c: I,
        address: Address,
        mut delay: D,
        config: &Configuration,
    ) -> Result<Self, Error<E>> {
        // 2 ms time to first communication (Datsheet Section 1, Table 2)
        delay.delay_ms(2);

        let mut data = [0; 2];
        i2c.write_read(
            address.into(),
            &[super::Register::CHIP_ID.into()],
            &mut data,
        )
        .map_err(Error::I2c)?;

        let chip_id = data[0];
        let rev_id = data[1];

        debug!("CHIP_ID = {=u8:#04x}; REV_ID = {=u8:#04x}", chip_id, rev_id);
        if chip_id != 0x60 {
            return Err(Error::WrongChip(chip_id));
        }

        // read Register::EVENT and INT_STATUS in a burst read to clear the event and interrupt status flags
        let mut data = [0; 2];
        i2c.write_read(address.into(), &[super::Register::EVENT.into()], &mut data)
            .map_err(Error::I2c)?;

        // write configuration after clearing interrupt status flags so that they are accurate from here on
        i2c.write(address.into(), &config.to_write_bytes())
            .map_err(Error::I2c)?;

        // read Register::ERR_REG after writing config to determine if configuration was successful and to clear the error status flags
        let mut err_reg = [0; 1];
        i2c.write_read(
            address.into(),
            &[super::Register::ERR_REG.into()],
            &mut err_reg,
        )
        .map_err(Error::I2c)
        .and_then(move |_| {
            let err_reg = super::ErrReg::from(err_reg[0]);
            if err_reg.fatal_err {
                return Err(Error::Fatal);
            } else if err_reg.cmd_err {
                return Err(Error::Command);
            } else if err_reg.conf_err {
                return Err(Error::Configuration);
            } else {
                Ok(())
            }
        })?;

        let coefficients = CalibrationCoefficients::try_from_i2c_sync(address, &mut i2c)?;

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
            altitude_reference: Length::new::<uom::si::length::meter>(0.0),
        }
    }

    /// Reads the temperature from the barometer as a [`ThermodynamicTemperature`].
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::sync::Bmp390;
    /// use uom::si::thermodynamic_temperature::degree_celsius;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let temperature = sensor.temperature()?;
    /// defmt::info!("Temperature: {} °C", temperature.get::<degree_celsius>());
    /// # Ok(())
    /// # }
    /// ```
    pub fn temperature(&mut self) -> Result<ThermodynamicTemperature, Error<E>> {
        // Burst read: only address DATA_3 (temperature XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_3.into()];
        let mut read = [0; 3];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
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
    /// # use bmp390::sync::Bmp390;
    /// use uom::si::pressure::hectopascal;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let pressure = sensor.pressure()?;
    /// defmt::info!("Pressure: {} hPa", pressure.get::<hectopascal>());
    /// # Ok(())
    /// # }
    /// ```
    pub fn pressure(&mut self) -> Result<Pressure, Error<E>> {
        // pressure requires temperature to compensate, so just measure both
        let measurement = self.measure()?;
        Ok(measurement.pressure)
    }

    /// Measures the pressure and temperature from the barometer.
    ///
    /// # Example
    /// ```no_run
    /// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
    /// # use bmp390::sync::Bmp390;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let measurement = sensor.measure()?;
    /// defmt::info!("Measurement: {}", measurement);
    /// # Ok(())
    /// # }
    /// ```
    pub fn measure(&mut self) -> Result<Measurement, Error<E>> {
        // Burst read: only address DATA_0 (pressure XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_0.into()];
        let mut read = [0; 6];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
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
    /// # use bmp390::sync::Bmp390;
    /// # use uom::si::length::meter;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let initial_altitude = sensor.altitude()?;
    /// sensor.set_reference_altitude(initial_altitude);
    ///
    /// // Some time later...
    /// let altitude = sensor.altitude()?;
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
    /// # use bmp390::sync::Bmp390;
    /// use uom::si::length::foot;
    /// # fn run() -> Result<(), bmp390::Error<embedded_hal::i2c::ErrorKind>> {
    /// # let config = bmp390::Configuration::default();
    /// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
    /// # let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config)?;
    /// let altitude = sensor.altitude()?;
    /// defmt::info!("Length: {} feet", altitude.get::<foot>());
    /// # Ok(())
    /// # }
    /// ```
    pub fn altitude(&mut self) -> Result<Length, Error<E>> {
        let pressure = self.pressure()?;
        Ok(calculate_altitude(pressure, self.altitude_reference))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use embedded_hal_mock::eh1::delay::{CheckedDelay, NoopDelay, Transaction as DelayTransaction};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as I2cTransaction};
    use std::prelude::rust_2021::*;
    use std::vec;
    use uom::si::length::meter;
    use uom::si::pressure::pascal;
    use uom::si::thermodynamic_temperature::degree_celsius;
    use uom::ConstZero;

    use crate::{ErrReg, Event, IntStatus};

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

    #[test]
    fn test_try_new() {
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
            DelayTransaction::delay_ms(2), // time to first communication
        ]);

        let _bmp390 = Bmp390::try_new(i2c.clone(), addr, delay.clone(), &config).unwrap();

        delay.done();
        i2c.done();
    }

    #[test]
    fn test_reads_temperature_and_compensates() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_3.into()],
            TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let temperature = bmp390.temperature().unwrap();
        assert_eq!(temperature, expected_temperature());
        i2c.done();
    }

    #[test]
    fn test_reads_pressure() {
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
        let pressure = bmp390.pressure().unwrap();
        assert_eq!(pressure, expected_pressure());
        i2c.done();
    }

    #[test]
    fn test_measure_reads_temperature_and_pressure() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            PRESSURE_TEMPERATURE_BYTES.to_vec(),
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());
        let measurement = bmp390.measure().unwrap();
        assert_eq!(measurement.temperature, expected_temperature());
        assert_eq!(measurement.pressure, expected_pressure());
        i2c.done();
    }

    #[test]
    fn test_altitude() {
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
        let altitude = bmp390.altitude().unwrap();
        assert_eq!(altitude, expected_altitude());
        i2c.done();
    }

    #[test]
    fn test_altitude_custom_reference() {
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
        let altitude = bmp390.altitude().unwrap();
        assert_eq!(altitude, Length::ZERO);
        i2c.done();
    }

    #[test]
    fn test_chip_id_incorrect() {
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
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default());
        assert!(matches!(result, Err(Error::WrongChip(0x42))));
        i2c.done();
    }

    #[test]
    fn test_fatal_error() {
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
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default());
        assert!(matches!(result, Err(Error::Fatal)));
        // assert_matches!(result, Err(Error::Fatal))); // TODO: use assert_matches once it's stable
        i2c.done();
    }

    #[test]
    fn test_command_error() {
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
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default());
        assert!(matches!(result, Err(Error::Command)));
        i2c.done();
    }

    #[test]
    fn test_configuration_error() {
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
        let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default());
        assert!(matches!(result, Err(Error::Configuration)));
        i2c.done();
    }

    #[test]
    fn test_any_other_error() {
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
            let result = Bmp390::try_new(i2c.clone(), addr, delay, &Configuration::default());
            assert!(
                result.is_ok(),
                "Unexpected error with ERR_REG = {:#010b}",
                err_reg_bits
            );

            i2c.done();
        }
    }
}
