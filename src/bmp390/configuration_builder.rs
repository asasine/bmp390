use super::Bmp390;
use crate::registers::{
    FifoConfig, FifoWatermark, FilterCoefficient, InterfaceConfig, InterruptControl,
    OutputDataRate, OversamplingConfig, PowerControl,
};
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use thiserror::Error;
use uom::si::f32::Length;

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
}

impl<I, E> Bmp390<I>
where
    I: RegisterInterface<AddressType = u8, Error = E>,
{
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{bmp390::test_utils::interface, raw};
    use core::assert_matches;
    use device_driver_mock::RegisterOperationRef::*;

    #[test]
    fn configure_writes_register_and_checks_for_errors() {
        let mut bmp390 = Bmp390::new(interface());
        let osr = OversamplingConfig::try_from(raw::field_sets::Osr::from([0x12])).unwrap();
        bmp390
            .configure(ConfigurationBuilder::new().oversampling(osr))
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

    #[test]
    fn configure_reports_invalid_configuration() {
        let interface = interface();
        let mut bmp390 = Bmp390::new(interface);

        // make the device report a configuration error
        bmp390
            .device()
            .err_reg()
            .write(|w| w.set_conf_err(true))
            .unwrap();

        let osr = OversamplingConfig::try_from(raw::field_sets::Osr::from([0x12])).unwrap();
        let result = bmp390.configure(ConfigurationBuilder::new().oversampling(osr));

        assert_eq!(result, Err(ConfigureError::RejectedByDevice));
    }
}
