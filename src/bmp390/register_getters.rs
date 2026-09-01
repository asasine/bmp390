//! Contains getters for the miscellaneous [`Bmp390`] registers.

use device_driver::{AsyncRegisterInterface, RegisterInterface};
use thiserror::Error;

use crate::registers::{
    ErrorStatus, Event, FilterCoefficient, InterfaceConfig, OutputDataRate, OversamplingConfig,
    PowerControl, ReservedValueError, Status,
};

pub use super::Bmp390;

/// An error that can occur when reading a register from the [`Bmp390`] sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegisterReadError<E> {
    /// An error originating from the underlying register interface.
    #[error(transparent)]
    Interface(E),

    /// A reserved value was encountered when reading the register.
    #[error(transparent)]
    ReservedValue(#[from] ReservedValueError),
}

impl<I, E> Bmp390<I>
where
    I: RegisterInterface<AddressType = u8, Error = E>,
{
    /// Read the chip identification code.
    pub fn chip_id(&mut self) -> Result<u8, E> {
        let chip_id = self.device.chip_id().read()?;
        Ok(chip_id.value())
    }

    /// Read the mask revision of the ASIC.
    pub fn rev_id(&mut self) -> Result<u8, E> {
        let rev_id = self.device.rev_id().read()?;
        Ok(rev_id.value())
    }

    /// Read the sensor error conditions. Cleared after reading.
    pub fn error(&mut self) -> Result<ErrorStatus, E> {
        let error = self.device.err_reg().read()?.into();
        Ok(error)
    }

    /// Read the sensor status flags.
    pub fn status(&mut self) -> Result<Status, E> {
        let status = self.device.status().read()?.into();
        Ok(status)
    }

    /// Read the sensor event flags. Cleared after reading.
    pub fn event(&mut self) -> Result<Event, E> {
        let event = self.device.event().read()?.into();
        Ok(event)
    }

    /// Read the serial interface settings.
    pub fn interface_config(&mut self) -> Result<InterfaceConfig, E> {
        let interface_config = self.device.if_conf().read()?.into();
        Ok(interface_config)
    }

    /// Read the sensor settings and measurement mode.
    pub fn power_control(&mut self) -> Result<PowerControl, E> {
        let power_control = self.device.pwr_ctrl().read()?.into();
        Ok(power_control)
    }

    /// Read the oversampling configuration.
    pub fn oversampling(&mut self) -> Result<OversamplingConfig, RegisterReadError<E>> {
        let oversampling = self
            .device
            .osr()
            .read()
            .map_err(RegisterReadError::Interface)?
            .try_into()?;

        Ok(oversampling)
    }

    /// Read the output data rate for pressure and temperature measurements.
    pub fn output_data_rate(&mut self) -> Result<OutputDataRate, RegisterReadError<E>> {
        let output_data_rate = self
            .device
            .odr()
            .read()
            .map_err(RegisterReadError::Interface)?
            .try_into()?;

        Ok(output_data_rate)
    }

    /// Read the IIR filter coefficient.
    pub fn filter_coefficient(&mut self) -> Result<FilterCoefficient, E> {
        let filter_coefficient = self.device.config().read()?.into();
        Ok(filter_coefficient)
    }
}

impl<I, E> Bmp390<I>
where
    I: AsyncRegisterInterface<AddressType = u8, Error = E>,
{
    /// Read the chip identification code.
    pub async fn chip_id_async(&mut self) -> Result<u8, E> {
        let chip_id = self.device.chip_id().read_async().await?;
        Ok(chip_id.value())
    }

    /// Read the mask revision of the ASIC.
    pub async fn rev_id_async(&mut self) -> Result<u8, E> {
        let rev_id = self.device.rev_id().read_async().await?;
        Ok(rev_id.value())
    }

    /// Read the sensor error conditions. Cleared after reading.
    pub async fn error_async(&mut self) -> Result<ErrorStatus, E> {
        let error = self.device.err_reg().read_async().await?.into();
        Ok(error)
    }

    /// Read the sensor status flags.
    pub async fn status_async(&mut self) -> Result<Status, E> {
        let status = self.device.status().read_async().await?.into();
        Ok(status)
    }

    /// Read the sensor event flags. Cleared after reading.
    pub async fn event_async(&mut self) -> Result<Event, E> {
        let event = self.device.event().read_async().await?.into();
        Ok(event)
    }

    /// Read the serial interface settings.
    pub async fn interface_config_async(&mut self) -> Result<InterfaceConfig, E> {
        let interface_config = self.device.if_conf().read_async().await?.into();
        Ok(interface_config)
    }

    /// Read the sensor settings and measurement mode.
    pub async fn power_control_async(&mut self) -> Result<PowerControl, E> {
        let power_control = self.device.pwr_ctrl().read_async().await?.into();
        Ok(power_control)
    }

    /// Read the oversampling configuration.
    pub async fn oversampling_async(&mut self) -> Result<OversamplingConfig, RegisterReadError<E>> {
        let oversampling = self
            .device
            .osr()
            .read_async()
            .await
            .map_err(RegisterReadError::Interface)?
            .try_into()?;

        Ok(oversampling)
    }

    /// Read the output data rate for pressure and temperature measurements.
    pub async fn output_data_rate_async(&mut self) -> Result<OutputDataRate, RegisterReadError<E>> {
        let output_data_rate = self
            .device
            .odr()
            .read_async()
            .await
            .map_err(RegisterReadError::Interface)?
            .try_into()?;

        Ok(output_data_rate)
    }

    /// Read the IIR filter coefficient.
    pub async fn filter_coefficient_async(&mut self) -> Result<FilterCoefficient, E> {
        let filter_coefficient = self.device.config().read_async().await?.into();
        Ok(filter_coefficient)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bmp390::test_utils::interface, raw};
    use core::assert_matches;

    #[test]
    fn reserved_oversampling_returns_error() {
        let mut bmp390 = Bmp390::new(interface());
        const VALUE: u8 = 0b110;
        bmp390.device.osr().write(|w| {
            let mut osr = raw::Osr::default();
            osr.set_pressure(raw::Oversampling::Reserved(VALUE));
            *w = osr;
        });

        let result = bmp390.oversampling();
        assert_matches!(
            result,
            Err(RegisterReadError::ReservedValue(ReservedValueError {
                value: VALUE,
                ..
            }))
        );
    }

    #[test]
    fn reserved_output_data_rate_returns_error() {
        let mut bmp390 = Bmp390::new(interface());
        const VALUE: u8 = 0x12;
        bmp390.device.odr().write(|w| {
            let mut odr = raw::Odr::default();
            odr.set_odr_sel(raw::OdrSel::Reserved(VALUE));
            *w = odr;
        });

        let result = bmp390.output_data_rate();
        assert_matches!(
            result,
            Err(RegisterReadError::ReservedValue(ReservedValueError {
                value: VALUE,
                ..
            }))
        );
    }
}
