use embedded_hal::i2c::Operation;

/// The BMP390 barometer's SDO value, which sets the I2C address.
///
/// The BMP390 can be configured to use two different addresses by either pulling the `SDO` pin down to `GND`
/// (`0x76` via [`Sdo::Down`]) or up to `V_DDIO` (`0x77` via [`Sdo::Up`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Sdo {
    /// `0x76`: The BMP390's address when `SDO` is pulled up to `GND`.
    Down,

    /// `0x77`: The BMP390's address when `SDO` is pulled down to `V_DDIO`
    Up,
}

impl From<Sdo> for u8 {
    /// Convert the address to a [`u8`] for I2C communication.
    fn from(sdo: Sdo) -> u8 {
        match sdo {
            Sdo::Down => 0x76,
            Sdo::Up => 0x77,
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Sdo {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Sdo::Down => defmt::write!(fmt, "Sdo::Down"),
            Sdo::Up => defmt::write!(fmt, "Sdo::Up"),
        }

        let address = u8::from(*self);
        defmt::write!(fmt, " (address: {=u8:#04X})", address);
    }
}

/// An I2C interface for the BMP390.
///
/// This type implements the [`device_driver`] interface traits for reading and writing
/// registers over I2C. The device's I2C address is determined by the [`Sdo`] pin.
///
/// # Example
/// Create a [`I2cInterface`] directly from the bus and address before passing it
/// to [`Bmp390::new`].
/// ```no_run
/// # use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::Mock};
/// use bmp390::dd::{Bmp390, Sdo, I2cInterface};
/// # async fn run() -> Result<(), embedded_hal_async::i2c::ErrorKind> {
/// # let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// # let delay = embedded_hal_mock::eh1::delay::NoopDelay::new();
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

/// The operations to write to a register, including selecting the register and writing the data.
fn write_ops<'a>(register_address: &'a u8, data: &'a [u8]) -> [Operation<'a>; 2] {
    [register_op(register_address), Operation::Write(data)]
}

/// The operations to read from a register, including selecting the register and reading the data.
fn read_ops<'a>(register_address: &'a u8, data: &'a mut [u8]) -> [Operation<'a>; 2] {
    [register_op(register_address), Operation::Read(data)]
}

impl<B: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface for I2cInterface<B> {
    type Error = B::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut operations = write_ops(&address, data);
        self.bus
            .transaction(self.address.into(), &mut operations)
            .await
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus
            .transaction(self.address.into(), &mut operations)
            .await
    }
}

impl<B: embedded_hal::i2c::I2c> device_driver::RegisterInterface for I2cInterface<B> {
    type Error = B::Error;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let mut operations = write_ops(&address, data);
        self.bus.transaction(self.address.into(), &mut operations)
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut operations = read_ops(&address, data);
        self.bus.transaction(self.address.into(), &mut operations)
    }
}
