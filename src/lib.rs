//! The BMP390 is a digital sensor with pressure and temperature measurement based on proven sensing principles. The
//! sensor is more accurate than its predecessor BMP380, covering a wider measurement range. It offers new interrupt
//! functionality, lower power consumption, and a new FIFO functionality. The integrated 512 byte FIFO buffer supports
//! low power applications and prevents data loss in non-real-time systems.
//!
//! [`Bmp390`] is a driver for the BMP390 sensor. It provides methods to read the temperature and pressure from the
//! sensor over [I2C](https://en.wikipedia.org/wiki/I%C2%B2C). It is built on top of the [`embedded_hal_async::i2c`]
//! traits to be compatible with a wide range of embedded platforms.
//!
//! # Example
//! ```no_run
//! # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
//! # async fn run() -> Result<(), bmp390::Bmp390Error<embedded_hal_async::i2c::ErrorKind>> {
//! use bmp390::Bmp390;
//! let config = bmp390::Configuration::default();
//! # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
//! let mut sensor = Bmp390::try_new(i2c, bmp390::Address::Up, delay, &config).await?;
//! let pressure = sensor.pressure().await?;
//! println!("Pressure: {:.2} hPa", pressure);
//! # Ok(())
//! # }
//! ```
//! # Datasheet
//! The [BMP390 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)
//! contains detailed information about the sensor's features, electrical characteristics, and registers. This package
//! implements the functionality described in the datasheet and references the relevant sections in the documentation.

#![cfg_attr(not(test), no_std)]

mod registers;

pub use registers::*;

use embedded_hal_async::{delay::DelayNs, i2c::I2c};

/// Errors that can occur when communicating with the BMP390 barometer.
#[derive(Debug, Clone, Copy)]
pub enum Bmp390Error<E> {
    /// An error occurred while communicating with the BMP390 over I2C. The inner error contains the specific error.
    I2c(E),

    /// A fatal error occurred on the BMP390. See [`ErrReg`] for more.
    Fatal,

    /// A command error occurred on the BMP390. See [`ErrReg`] for more.
    Command,

    /// A configuration error occurred on the BMP390. See [`ErrReg`] for more.
    Configuration,
}

impl From<embedded_hal_async::i2c::ErrorKind> for Bmp390Error<embedded_hal_async::i2c::ErrorKind> {
    fn from(error: embedded_hal_async::i2c::ErrorKind) -> Self {
        Bmp390Error::I2c(error)
    }
}

/// A single measurement from the BMP390 barometer.
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    /// The pressure in hectopascals (hPa).
    pub pressure: f32,

    /// The temperature in degrees Celsius (°C).
    pub temperature: f32,
}

/// The BMP390 barometer's I2C addresses, either `0x76` or `0x77`.
///
///  The BMP390 can be configured to use two different addresses by either pulling the `SDO` pin down to `GND`
/// (`0x76` via [`Address::Down`]) or up to `V_DDIO` (`0x77` via [`Address::Up`]).
#[derive(Debug, Clone, Copy)]
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
    async fn try_from_i2c<I: I2c>(
        address: Address,
        i2c: &mut I,
    ) -> Result<Self, Bmp390Error<I::Error>> {
        let mut calibration_coefficient_regs = [0; 21];
        i2c.write_read(
            address.into(),
            &Self::write_read_write_transaction(),
            &mut calibration_coefficient_regs,
        )
        .await
        .map_err(Bmp390Error::I2c)?;

        Ok(Self::from_registers(&calibration_coefficient_regs))
    }

    /// Calculate the calibration coefficients from the raw register data in registers [`Register::NVM_PAR_T1_0`] to
    /// [`Register::NVM_PAR_P11`].
    ///
    /// See: Datasheet Apendix A, Section 8.4
    fn from_registers(data: &[u8; 21]) -> Self {
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
            par_p4: (nvm_par_p4 as f32) / 137_438_953_427.0, // 2^37
            par_p5: (nvm_par_p5 as f32) / 0.125,           // 2^-3
            par_p6: (nvm_par_p6 as f32) / 64.0,            // 2^6
            par_p7: (nvm_par_p7 as f32) / 256.0,           // 2^8
            par_p8: (nvm_par_p8 as f32) / 32768.0,         // 2^15
            par_p9: (nvm_par_p9 as f32) / 81_474_976_710_656.0, //2^48
            par_p10: (nvm_par_p10 as f32) / 81_474_976_710_656.0, // 2^48
            par_p11: (nvm_par_p11 as f32) / 36_893_488_147_419_103_232.0, // 2^65
        }
    }

    /// Compensate a temperature reading according to calibration coefficients.
    ///
    /// See: Datasheet Apendix A, Section 8.5
    fn compensate_temperature(&self, temperature_uncompensated: i32) -> f32 {
        let uncompensated = temperature_uncompensated as f32;
        let partial_1 = uncompensated - self.par_t1;
        let partial_2 = partial_1 * self.par_t2;
        partial_2 + (partial_1 * partial_1) * self.par_t3
    }

    /// Compensate a pressure reading according to calibration coefficients.
    ///
    /// See: Datasheet Apendix A, Section 8.6
    fn compensate_pressure(&self, tempature: f32, pressure_uncompensated: i32) -> f32 {
        let uncompensated = pressure_uncompensated as f32;
        let partial_1 = self.par_p6 * tempature;
        let partial_2 = self.par_p7 * tempature * tempature;
        let partial_3 = self.par_p8 * tempature * tempature * tempature;
        let partial_out1 = self.par_p5 + partial_1 + partial_2 + partial_3;

        let partial_1 = self.par_p2 * tempature;
        let partial_2 = self.par_p3 * tempature * tempature;
        let partial_3 = self.par_p4 * tempature * tempature * tempature;
        let partial_out2 = uncompensated * (self.par_p1 + partial_1 + partial_2 + partial_3);

        let partial_1 = uncompensated * uncompensated;
        let partial_2 = self.par_p9 + self.par_p10 * tempature;
        let partial_3 = partial_1 * partial_2;
        let partial_4 = partial_3 + tempature * tempature * tempature * tempature * self.par_p11;

        partial_out1 + partial_out2 + partial_4
    }

    /// Gets the bytes to write in a write-read transaction to the BMP390 to read the calibration coefficients.
    fn write_read_write_transaction() -> [u8; 21] {
        [
            Register::NVM_PAR_T1_0.into(),
            Register::NVM_PAR_T1_1.into(),
            Register::NVM_PAR_T2_0.into(),
            Register::NVM_PAR_T2_1.into(),
            Register::NVM_PAR_T3.into(),
            Register::NVM_PAR_P1_0.into(),
            Register::NVM_PAR_P1_1.into(),
            Register::NVM_PAR_P2_0.into(),
            Register::NVM_PAR_P2_1.into(),
            Register::NVM_PAR_P3.into(),
            Register::NVM_PAR_P4.into(),
            Register::NVM_PAR_P5_0.into(),
            Register::NVM_PAR_P5_1.into(),
            Register::NVM_PAR_P6_0.into(),
            Register::NVM_PAR_P6_1.into(),
            Register::NVM_PAR_P7.into(),
            Register::NVM_PAR_P8.into(),
            Register::NVM_PAR_P9_0.into(),
            Register::NVM_PAR_P9_1.into(),
            Register::NVM_PAR_P10.into(),
            Register::NVM_PAR_P11.into(),
        ]
    }
}

/// Configuration for the BMP390 barometer.
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

/// BMP390 barometer driver.
pub struct Bmp390<I> {
    /// The I2C bus the barometer is connected to.
    i2c: I,

    /// The I2C address of the barometer.
    address: Address,

    /// The calibration coefficients for the barometer to compensate temperature and pressure measurements.
    coefficients: CalibrationCoefficients,
}

impl<I, E> Bmp390<I>
where
    I: I2c<Error = E>,
{
    /// Creates a new BMP390 driver. This will initialize the barometer with the provided configuration.
    /// It will additionally delay for 2 ms to allow the barometer to start up and read the calibration coefficients
    /// for temperature and pressure measuring.
    pub async fn try_new<D: DelayNs>(
        mut i2c: I,
        address: Address,
        mut delay: D,
        config: &Configuration,
    ) -> Result<Self, Bmp390Error<E>> {
        // 2 ms time to first communication (Datsheet Section 1, Table 2)
        delay.delay_ms(2).await;

        // read Register::EVENT to clear the event status flags
        i2c.write_read(address.into(), &[Register::EVENT.into()], &mut [0; 1])
            .await
            .map_err(Bmp390Error::I2c)?;

        // read Register::INT_STATUS to clear the interrupt status flags
        i2c.write_read(address.into(), &[Register::INT_STATUS.into()], &mut [0; 1])
            .await
            .map_err(Bmp390Error::I2c)?;

        // write configuration after clearing interrupt status flags so that they are accurate from here on
        i2c.write(address.into(), &config.to_write_bytes())
            .await
            .map_err(Bmp390Error::I2c)?;

        // read Register::ERR_REG to determine if configuration was successful and to clear the error status flags
        let mut err_reg = [0; 1];
        i2c.write_read(address.into(), &[Register::ERR_REG.into()], &mut err_reg)
            .await
            .map_err(Bmp390Error::I2c)
            .and_then(move |_| {
                let err_reg = ErrReg::from(err_reg[0]);
                if err_reg.fatal_err {
                    Err(Bmp390Error::<E>::Fatal)
                } else if err_reg.cmd_err {
                    Err(Bmp390Error::<E>::Command)
                } else if err_reg.conf_err {
                    Err(Bmp390Error::<E>::Configuration)
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
        }
    }

    /// Reads the temperature from the barometer.
    pub async fn temperature(&mut self) -> Result<f32, Bmp390Error<E>> {
        // Burst read: only address DATA_3 (temperature XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_3.into()];
        let mut read = [0; 3];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
            .await
            .map_err(Bmp390Error::I2c)?;

        // DATA_3 is the LSB, DATA_5 is the MSB
        let temerpature_uncompensated =
            i32::from(read[0]) | i32::from(read[1]) << 8 | i32::from(read[2]) << 16;

        let temerpature = self
            .coefficients
            .compensate_temperature(temerpature_uncompensated);

        Ok(temerpature)
    }

    /// Reads the pressure from the barometer.
    pub async fn pressure(&mut self) -> Result<f32, Bmp390Error<E>> {
        // pressure requires temperature to compensate, so just measure both
        let measurement = self.measure().await?;
        Ok(measurement.pressure)
    }

    pub async fn measure(&mut self) -> Result<Measurement, Bmp390Error<E>> {
        // Burst read: only address DATA_0 (pressure XLSB) and BMP390 auto-increments through DATA_5 (temperature MSB)
        let write = &[Register::DATA_0.into()];
        let mut read = [0; 6];
        self.i2c
            .write_read(self.address.into(), write, &mut read)
            .await
            .map_err(Bmp390Error::I2c)?;

        // pressure is 0:2 (XLSB, LSB, MSB), temperature is 3:5 (XLSB, LSB, MSB)
        let temperature_uncompensated =
            i32::from(read[3]) | i32::from(read[4]) << 8 | i32::from(read[5]) << 16;
        let temperature = self
            .coefficients
            .compensate_temperature(temperature_uncompensated);

        let pressure_uncompensated =
            i32::from(read[0]) | i32::from(read[1]) << 8 | i32::from(read[2]) << 16;

        let pressure = self
            .coefficients
            .compensate_pressure(temperature, pressure_uncompensated);

        Ok(Measurement {
            temperature,
            pressure,
        })
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::delay::{CheckedDelay, NoopDelay, Transaction as DelayTransaction};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction as I2cTransaction};

    use super::*;

    impl Default for CalibrationCoefficients {
        fn default() -> Self {
            Self {
                par_t1: 0.0,
                par_t2: 0.0,
                par_t3: 0.0,
                par_p1: 0.0,
                par_p2: 0.0,
                par_p3: 0.0,
                par_p4: 0.0,
                par_p5: 0.0,
                par_p6: 0.0,
                par_p7: 0.0,
                par_p8: 0.0,
                par_p9: 0.0,
                par_p10: 0.0,
                par_p11: 0.0,
            }
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
            I2cTransaction::write_read(
                addr.into(),
                vec![Register::EVENT.into()],
                vec![u8::from(*event)],
            ),
            I2cTransaction::write_read(
                addr.into(),
                vec![Register::INT_STATUS.into()],
                vec![u8::from(*int_status)],
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
    async fn test_reads_compensation_coefficients_and_writes_configuration() {
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
    async fn test_reads_temperature() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_3.into()],
            vec![0; 3],
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());

        let _temperature = bmp390.temperature().await.unwrap();
        i2c.done();
    }

    #[tokio::test]
    async fn test_reads_pressure() {
        let addr = Address::Up;

        // NOTE: a pressure read requires a temperature read, so response is 6 bytes
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            vec![0; 6],
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());

        let _pressure = bmp390.pressure().await.unwrap();
        i2c.done();
    }

    #[tokio::test]
    async fn test_measure_reads_temperature_and_pressure() {
        let addr = Address::Up;
        let expectations = [I2cTransaction::write_read(
            addr.into(),
            vec![Register::DATA_0.into()],
            vec![0; 6],
        )];

        let mut i2c = Mock::new(&expectations);
        let mut bmp390 =
            Bmp390::new_with_coefficients(i2c.clone(), addr, CalibrationCoefficients::default());

        let _measurement = bmp390.measure().await.unwrap();
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
        assert!(matches!(result, Err(Bmp390Error::Fatal)));
        // assert_matches!(result, Err(Bmp390Error::Fatal))); // TODO: use assert_matches once it's stable
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
        assert!(matches!(result, Err(Bmp390Error::Command)));
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
        assert!(matches!(result, Err(Bmp390Error::Configuration)));
        i2c.done();
    }

    #[tokio::test]
    async fn test_any_other_error() {
        // Test how the driver handles unexpected bits in the ERR_REG register
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

    #[tokio::test]
    async fn test_compensation() {
        // TODO: test that the temperature and pressure registers are read and compensated correctly
    }
}
