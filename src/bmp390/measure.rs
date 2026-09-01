use super::Bmp390;
use crate::{Measurement, measurement::calculate_altitude, registers::Coefficients};
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use uom::si::f32::{Length, Pressure, ThermodynamicTemperature};

impl<I> Bmp390<I> {
    /// Set the reference altitude for altitude calculations.
    ///
    /// Following this, the altitude can be calculated using [`Self::altitude`]. If the current pressure matches
    /// the pressure when the reference altitude is set, the altitude will be 0.
    pub fn set_reference_altitude(&mut self, altitude: Length) {
        self.reference_altitude = altitude;
    }
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

    /// Get the sensor time.
    ///
    /// This is an internal timestamp that can be used to correlate measurments.
    pub async fn sensor_time_async(&mut self) -> Result<u32, E> {
        let sensor_time = self.device.sensor_time().read_async().await?;
        Ok(sensor_time.value())
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

    /// Get the sensor time.
    ///
    /// This is an internal timestamp that can be used to correlate measurements.
    pub fn sensor_time(&mut self) -> Result<u32, E> {
        let sensor_time = self.device.sensor_time().read()?;
        Ok(sensor_time.value())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bmp390::test_utils::interface;
    use core::assert_matches;
    use device_driver::Block;
    use device_driver_mock::{RegisterOperation, RegisterOperationRef::*};
    use uom::{
        ConstZero,
        si::{length::meter, pressure::pascal, thermodynamic_temperature::degree_celsius},
    };

    fn expected_pressure() -> Pressure {
        Pressure::new::<pascal>(98370.55)
    }

    fn expected_temperature() -> ThermodynamicTemperature {
        ThermodynamicTemperature::new::<degree_celsius>(25.770_746)
    }

    fn expected_altitude() -> Length {
        Length::new::<meter>(248.78754)
    }

    #[test]
    fn reads_temperature_and_compensates() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(temperature) = bmp390.temperature();
        assert_eq!(temperature, expected_temperature());
        assert_matches!(
            bmp390.device.interface().ops_as_ref().as_slice(),
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

    #[test]
    fn reads_pressure() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(pressure) = bmp390.pressure();
        assert_eq!(pressure, expected_pressure());
    }

    #[test]
    fn reads_temperature_and_pressure() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(measurement) = bmp390.temperature_and_pressure();
        assert_eq!(measurement.0, expected_temperature());
        assert_eq!(measurement.1, expected_pressure());
    }

    #[test]
    fn reads_altitude() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(altitude) = bmp390.altitude();
        assert_eq!(altitude, expected_altitude());
    }

    #[test]
    fn measure_reads_temperature_pressure_and_altitude() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(measurement) = bmp390.measure();
        assert_eq!(measurement.temperature, expected_temperature());
        assert_eq!(measurement.pressure, expected_pressure());
        assert_eq!(measurement.altitude, expected_altitude());
    }

    #[test]
    fn altitude_uses_custom_reference() {
        let mut bmp390 = Bmp390::new(interface());
        bmp390.set_reference_altitude(expected_altitude());
        let Ok(altitude) = bmp390.altitude();
        assert_eq!(altitude, Length::ZERO);
    }

    #[test]
    fn caches_calibration_coefficients() {
        let mut bmp390 = Bmp390::new(interface());
        let Ok(_) = bmp390.temperature();
        let Ok(_) = bmp390.temperature();
        let calibration_reads = bmp390
            .device
            .interface()
            .operations
            .iter()
            .filter(|operation| matches!(operation, RegisterOperation::Read { address: 0x30, .. }))
            .count();

        assert_eq!(calibration_reads, 1);
    }
}
