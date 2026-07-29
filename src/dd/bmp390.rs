use super::Device;
use core::slice;
use embedded_hal::i2c::Operation;

/// The BMP390 barometer's SDO value, which sets the I2C address.*
///
///  The BMP390 can be configured to use two different addresses by either pulling the `SDO` pin down to `GND`
/// (`0x76` via [`Sdo::Down`]) or up to `V_DDIO` (`0x77` via [`Sdo::Up`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sdo {
    /// `0x76`: The BMP390's address when `SDO` is pulled up to `GND`.
    Down = 0x76,

    /// `0x77`: The BMP390's address when `SDO` is pulled down to `V_DDIO`
    Up = 0x77,
}

impl From<Sdo> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(sdo: Sdo) -> u8 {
        sdo as u8
    }
}

/// The BMP390 device driver.
#[derive(Debug)]
pub struct Bmp390<B> {
    bus: B,
    address: u8,
}

impl<B> Bmp390<B> {
    /// Create a new [`Bmp390`] with the given bus and address derived from the `SDO` pin.
    pub fn new(bus: B, sdo: Sdo) -> Self {
        Self::new_with_address(bus, sdo.into())
    }

    /// Create a new [`Bmp390`] with the given bus and address.
    pub fn new_with_address(bus: B, address: u8) -> Self {
        Self { bus, address }
    }

    /// Get a [`Device`] instance, which can be used to read and write registers on the device.
    ///
    /// This instance may be freely used and dropped.
    pub fn device(&mut self) -> Device<&mut Self> {
        Device::new(self)
    }
}

/// The operations to select a register for reading or writing.
fn register_op<'a>(register_address: &'a u8) -> Operation<'a> {
    Operation::Write(slice::from_ref(register_address))
}

/// The operations to write to a register, including selecting the register and writing the data.
fn write_ops<'a>(register_address: &'a u8, data: &'a [u8]) -> [Operation<'a>; 2] {
    [register_op(register_address), Operation::Write(data)]
}

/// The operations to read from a register, including selecting the register and reading the data.
fn read_ops<'a>(register_address: &'a u8, data: &'a mut [u8]) -> [Operation<'a>; 2] {
    [register_op(register_address), Operation::Read(data)]
}

impl<B: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for &mut Bmp390<B> {
    type Error = B::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut operations = write_ops(&address, data);
        self.bus.transaction(self.address, &mut operations).await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus.transaction(self.address, &mut operations).await
    }
}

#[cfg(feature = "sync")]
impl<B: embedded_hal::i2c::I2c> device_driver::RegisterInterface for &mut Bmp390<B> {
    type Error = B::Error;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut operations = write_ops(&address, data);
        self.bus.transaction(self.address, &mut operations)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus.transaction(self.address, &mut operations)
    }
}
