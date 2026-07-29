
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
