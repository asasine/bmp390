//! Command support for [`Bmp390`]

//! Contains getters for the miscellaneous [`Bmp390`] registers.

use crate::registers::Command;
use device_driver::{AsyncCommandInterface, CommandInterface};

pub use super::Bmp390;

impl<I, E> Bmp390<I>
where
    I: CommandInterface<AddressType = u8, Error = E>,
{
    /// Execute a command on the device.
    pub fn execute(&mut self, command: Command) -> Result<(), E> {
        self.device
            .cmd()
            .dispatch_in(|cmd| cmd.set_cmd(command.into()))
    }
}

impl<I, E> Bmp390<I>
where
    I: AsyncCommandInterface<AddressType = u8, Error = E>,
{
    /// Execute a command on the device.
    pub async fn execute_async(&mut self, command: Command) -> Result<(), E> {
        self.device
            .cmd()
            .dispatch_in_async(|cmd| cmd.set_cmd(command.into()))
            .await
    }
}
