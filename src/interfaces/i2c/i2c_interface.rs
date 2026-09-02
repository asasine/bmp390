use super::Sdo;
use device_driver::{
    AsyncRegisterInterface, FieldsetMetadata, RegisterInterface, RegisterInterfaceBase,
};
use embedded_hal::i2c::{self, I2c, Operation};
use embedded_hal_async::i2c::I2c as AsyncI2c;

/// An I2C interface for the BMP390.
///
/// This type implements the [`device_driver`] interface traits for reading and writing
/// registers over I2C. The device's I2C address is determined by the [`Sdo`] pin.
///
/// To invoke commands, wrap this in a [`Polling`](crate::interfaces::Polling) and provide a delay
/// provider to poll the device for command completion.
///
/// # Example
/// Create a [`I2cInterface`] directly from the bus and address before passing it
/// to [`Bmp390::new`].
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
/// # Ok(())
/// # }
/// ```
pub struct I2cInterface<B> {
    /// The device's bus. This should implement [`embedded_hal::i2c::I2c`] and/or
    /// [`embedded_hal_async::i2c::I2c`] depending on whether the synchronous and/or
    /// asynchronous interface is used.
    pub bus: B,

    /// The pull direction of the `SDO` pin, which determines the I2C address of
    /// the device on the [`bus`][Self::bus].
    pub address: Sdo,
}

#[cfg(feature = "defmt")]
impl<B: defmt::Format> defmt::Format for I2cInterface<B> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "I2cInterface {{ bus: {}, address: {} }}",
            self.bus,
            self.address
        );
    }
}

/// The operations to select a register for reading or writing.
fn register_op<'a>(register_address: &'a u8) -> Operation<'a> {
    Operation::Write(core::slice::from_ref(register_address))
}

/// The operations to read from a register, including selecting the register and reading the data.
fn read_ops<'a>(register_address: &'a u8, data: &'a mut [u8]) -> [Operation<'a>; 2] {
    [register_op(register_address), Operation::Read(data)]
}

impl<B: i2c::ErrorType> RegisterInterfaceBase for I2cInterface<B> {
    type Error = B::Error;
    type AddressType = u8;
}

impl<B: AsyncI2c> AsyncRegisterInterface for I2cInterface<B> {
    async fn write_register(
        &mut self,
        mut address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        for value in data.iter().copied() {
            self.bus
                .write(self.address.into(), &[address, value])
                .await?;

            address += 1;
        }

        Ok(())
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus
            .transaction(self.address.into(), &mut operations)
            .await
    }
}

impl<B: I2c> RegisterInterface for I2cInterface<B> {
    fn write_register(
        &mut self,
        mut address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        for value in data.iter().copied() {
            self.bus.write(self.address.into(), &[address, value])?;
            address += 1;
        }

        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus.transaction(self.address.into(), &mut operations)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    #[test]
    fn multi_byte_writes_use_register_data_pairs() {
        let address = Sdo::Up;
        let expectations = [
            Transaction::write(address.into(), std::vec![0x15, 0x11]),
            Transaction::write(address.into(), std::vec![0x16, 0x01]),
        ];

        let mut interface = I2cInterface {
            bus: Mock::new(&expectations),
            address,
        };
        let mut data = [0x11, 0x01];

        RegisterInterface::write_register(
            &mut interface,
            0x15,
            &mut data,
            &FieldsetMetadata::new(),
        )
        .unwrap();
        interface.bus.done();
    }
}
