use crate::raw::{CommandStatus, Device};
use device_driver::{
    AsyncCommandInterface, AsyncRegisterInterface, CommandInterface, CommandInterfaceBase,
    FieldsetMetadata, RegisterInterface, RegisterInterfaceBase,
};
use embedded_hal::delay::DelayNs;
use embedded_hal_async::delay::DelayNs as AsyncDelayNs;
use thiserror::Error;

/// An I2C interface that provides command dispatching by polling the device for command completion.
///
/// [`Polling`] wraps any [`RegisterInterface`]/[`AsyncRegisterInterface`] implementation
/// to implement [`CommandInterface`]/[`AsyncCommandInterface`] by polling the device's [`STATUS`]
/// register until the command is complete, then checking the [`ERR_REG`] register
/// for any errors.
///
/// [`STATUS`]: crate::Status
/// [`ERR_REG`]: crate::ErrReg
pub struct Polling<I, D> {
    /// The underlying register interface implementation.
    pub interface: I,

    /// A delay provider for the device, used to poll the device for command completion.
    ///
    /// If commands are not used, this can be set to any type such as [`()`]
    pub delay: D,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Error)]
pub enum CommandError<E> {
    /// An error occurred in the underlying interface.
    #[error(transparent)]
    Interface(#[from] E),

    /// The command failed, as indicated by the device's `ERR_REG` register.
    #[error("The command failed, as indicated by the device's `ERR_REG` register.")]
    CommandFailed,
}

impl<I: RegisterInterfaceBase, D> CommandInterfaceBase for Polling<I, D> {
    type Error = CommandError<I::Error>;
    type AddressType = I::AddressType;
}

/// Implements command dispatching for the BMP390 over I2C by polling the `STATUS`
/// register until the command is complete, then checking the `ERR_REG` register
/// for any errors.
impl<I, D> AsyncCommandInterface for Polling<I, D>
where
    I: AsyncRegisterInterface<AddressType = u8>,
    D: AsyncDelayNs,
{
    /// Dispatch a command to the BMP390 device.
    ///
    /// # Arguments
    /// - `address`: The command register's address. Should always be `CMD` (0x7E)
    ///   for the BMP390.
    /// - `input`: The input data to send to the command register. Should always
    ///   be a single byte of the command to execute.
    /// - `input_metadata`: Metadata for the input fieldset.
    /// - `_output`: Unused; the BMP390 does not return any data for commands.
    /// - `_output_metadata`: Unused; metadata for the output fieldset.
    async fn dispatch_command(
        &mut self,
        address: Self::AddressType,
        input: &mut [u8],
        input_metadata: &FieldsetMetadata,
        _output: &mut [u8],
        _output_metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        // write the command to the CMD register
        self.write_register(address, input, input_metadata).await?;

        // poll for command completion
        let mut regs = Device::new(&mut self.interface);
        loop {
            let status = regs.status().read_async().await?;
            if status.command_ready() == CommandStatus::Ready {
                break;
            }

            self.delay.delay_ms(2).await;
        }

        // check for errors
        let err_reg = regs.err_reg().read_async().await?;
        if err_reg.cmd_err() {
            Err(CommandError::CommandFailed)
        } else {
            Ok(())
        }
    }
}

impl<I, D> CommandInterface for Polling<I, D>
where
    I: RegisterInterface<AddressType = u8>,
    D: DelayNs,
{
    fn dispatch_command(
        &mut self,
        address: Self::AddressType,
        input: &mut [u8],
        input_metadata: &FieldsetMetadata,
        _output: &mut [u8],
        _output_metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        // write the command to the CMD register
        self.write_register(address, input, input_metadata)?;

        let mut regs = Device::new(&mut self.interface);
        loop {
            let status = regs.status().read()?;
            if status.command_ready() == CommandStatus::Ready {
                break;
            }

            self.delay.delay_ms(2);
        }

        let err_reg = regs.err_reg().read()?;
        if err_reg.cmd_err() {
            Err(CommandError::CommandFailed)
        } else {
            Ok(())
        }
    }
}

impl<I: RegisterInterfaceBase, D> RegisterInterfaceBase for Polling<I, D> {
    type Error = I::Error;
    type AddressType = I::AddressType;
}

/// Supports the same register interface as the underlying interface in addition
/// to command dispatching via the [`CommandInterface`] trait.
impl<I: AsyncRegisterInterface, D> AsyncRegisterInterface for Polling<I, D> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        metadata: &FieldsetMetadata,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.interface.write_register(address, data, metadata)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        metadata: &FieldsetMetadata,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        self.interface.read_register(address, data, metadata)
    }
}

/// Supports the same register interface as the underlying interface in addition
/// to command dispatching via the [`CommandInterface`] trait.
impl<I: RegisterInterface, D> RegisterInterface for Polling<I, D> {
    fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.interface.write_register(address, data, metadata)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        self.interface.read_register(address, data, metadata)
    }
}
