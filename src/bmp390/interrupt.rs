//! [`Bmp390`] interrupt handling.

use device_driver::{AsyncRegisterInterface, RegisterInterface};

use crate::registers::{InterruptControl, InterruptStatus};

pub use super::Bmp390;

impl<I, E> Bmp390<I>
where
    I: RegisterInterface<AddressType = u8, Error = E>,
{
    /// Read the sensor interrupt status flags. Cleared after reading.
    pub fn interrupt_status(&mut self) -> Result<InterruptStatus, E> {
        let interrupt_status = self.device.int_status().read()?.into();
        Ok(interrupt_status)
    }

    /// Read the interrupt configuration.
    pub fn interrupt_control(&mut self) -> Result<InterruptControl, E> {
        let interrupt_control = self.device.int_ctrl().read()?.into();
        Ok(interrupt_control)
    }
}

impl<I, E> Bmp390<I>
where
    I: AsyncRegisterInterface<AddressType = u8, Error = E>,
{
    /// Read the sensor interrupt status flags. Cleared after reading.
    pub async fn interrupt_status_async(&mut self) -> Result<InterruptStatus, E> {
        let interrupt_status = self.device.int_status().read_async().await?.into();
        Ok(interrupt_status)
    }

    /// Read the interrupt configuration.
    pub async fn interrupt_control_async(&mut self) -> Result<InterruptControl, E> {
        let interrupt_control = self.device.int_ctrl().read_async().await?.into();
        Ok(interrupt_control)
    }
}
