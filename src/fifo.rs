/// The header of a FIFO frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FifoHeader {
    /// The FIFO frame contains sensor data.
    SensorFrame(EnabledSensors),

    /// The FIFO frame contains a configuration error.
    ConfigurationError,

    /// The FIFO input configuration has changed.
    FifoInputConfigurationChange,
}

/// The sensors that are included in the data of this FIFO frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct EnabledSensors {
    /// The data includes the time.
    time: bool,

    /// The data includes the temperature.
    temperature: bool,

    /// The data includes the pressure.
    pressure: bool,
}

impl From<u8> for FifoHeader {
    fn from(value: u8) -> Self {
        let mode = (value >> 6) & 0b11;
        let param = (value >> 2) & 0b1111;
        match mode {
            0b10 => FifoHeader::SensorFrame(EnabledSensors {
                time: param & 0b1000 != 0,
                temperature: param & 0b0100 != 0,
                pressure: param & 0b0001 != 0,
            }),
            0b01 => match param {
                0b0001 => FifoHeader::ConfigurationError,
                0b0010 => FifoHeader::FifoInputConfigurationChange,
                _ => panic!("Invalid parameter: {:#04X}", param),
            },
            _ => panic!("Invalid mode: {:#04X}", mode),
        }
    }
}

impl From<&u8> for FifoHeader {
    fn from(value: &u8) -> Self {
        Self::from(*value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FifoData {
    /// The FIFO frame contains time data.
    Time(u32),

    /// The FIFO frame contains temperature data.
    Temperature(u32),

    /// The FIFO frame contains pressure data.
    Pressure(u32),

    /// The FIFO frame contains temperature and pressure data.
    TemperatureAndPressure(u32, u32),

    /// The FIFO frame contains a configuration error.
    ///
    /// The data bytes for this is always an opcode of `0x01` therefore the opcode is not returned.
    ConfigurationError,

    /// The FIFO input configuration has changed.
    ///
    /// The data bytes for this is always an opcode of `0x01` therefore the opcode is not returned.
    FifoInputConfigurationChange,
}

/// An iterator over FIFO frames.
pub struct FifoIter<T> {
    iter: T,
}

impl<T: core::iter::Iterator<Item = u8>> FifoIter<T> {
    pub(crate) fn new(iter: T) -> Self {
        Self { iter }
    }

    /// Consumes three bytes from [`Self::iter`] and returns it as a [`u32`] with the most significant byte as 0.
    fn next_u24(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes([
            self.iter.next()?,
            self.iter.next()?,
            self.iter.next()?,
            0,
        ]))
    }
}

impl<T: core::iter::Iterator<Item = u8>> Iterator for FifoIter<T> {
    type Item = FifoData;

    fn next(&mut self) -> Option<Self::Item> {
        let header = self.iter.next().map(FifoHeader::from)?;
        match header {
            FifoHeader::SensorFrame(EnabledSensors {
                time,
                temperature,
                pressure,
            }) => {
                if time {
                    Some(FifoData::Time(self.next_u24()?))
                } else if temperature && pressure {
                    Some(FifoData::TemperatureAndPressure(
                        self.next_u24()?,
                        self.next_u24()?,
                    ))
                } else if temperature {
                    Some(FifoData::Temperature(self.next_u24()?))
                } else if pressure {
                    Some(FifoData::Pressure(self.next_u24()?))
                } else {
                    // an empty sensor frame indicates we've exhausted the FIFO so consume the rest of the buffer
                    while let Some(_) = self.iter.next() {}
                    None
                }
            }
            FifoHeader::ConfigurationError => {
                self.iter.next(); // the opcode is always 0x01 so we don't need to return it
                Some(FifoData::ConfigurationError)
            }
            FifoHeader::FifoInputConfigurationChange => {
                self.iter.next(); // the opcode is always 0x01 so we don't need to return it
                Some(FifoData::FifoInputConfigurationChange)
            }
        }
    }
}

impl<T: core::iter::Iterator<Item = u8>> core::iter::FusedIterator for FifoIter<T> {}

#[cfg(test)]
mod tests {
    use super::*;

    mod header {
        use super::*;

        #[test]
        fn empty() {
            let header = FifoHeader::from(0b10_0000_00);
            assert_eq!(
                header,
                FifoHeader::SensorFrame(EnabledSensors {
                    time: false,
                    temperature: false,
                    pressure: false,
                })
            );
        }

        #[test]
        fn time() {
            let header = FifoHeader::from(0b10_1000_00);
            assert_eq!(
                header,
                FifoHeader::SensorFrame(EnabledSensors {
                    time: true,
                    temperature: false,
                    pressure: false,
                })
            );
        }

        #[test]
        fn pressure_and_temperature() {
            let header = FifoHeader::from(0b10_0101_00);
            assert_eq!(
                header,
                FifoHeader::SensorFrame(EnabledSensors {
                    time: false,
                    temperature: true,
                    pressure: true,
                })
            );
        }

        #[test]
        fn configuration_error() {
            let header = FifoHeader::from(0b01_0001_00);
            assert_eq!(header, FifoHeader::ConfigurationError);
        }

        #[test]
        fn fifo_input_configuration_change() {
            let header = FifoHeader::from(0b01_0010_00);
            assert_eq!(header, FifoHeader::FifoInputConfigurationChange);
        }
    }

    mod iter {
        use super::*;
        #[test]
        fn empty() {
            let data = [0b10_0000_00, 0];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), None);
            assert_eq!(iter.next(), None, "the iterator should be fused");
        }

        #[test]
        fn time() {
            let data = [0b10_1000_00, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::Time(0x00112233)));
            assert_eq!(iter.next(), None);
        }

        #[test]
        fn temperature() {
            let data = [0b10_0100_00, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::Temperature(0x00112233)));
            assert_eq!(iter.next(), None);
        }

        #[test]
        fn pressure() {
            let data = [0b10_0001_00, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::Pressure(0x00112233)));
            assert_eq!(iter.next(), None);
        }

        #[test]
        fn temperature_and_pressure() {
            let data = [0b10_0101_00, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(
                iter.next(),
                Some(FifoData::TemperatureAndPressure(0x00445566, 0x00112233))
            );

            assert_eq!(iter.next(), None);
        }

        #[test]
        fn multiple() {
            let data = [
                0b10_0101_00,
                0x66,
                0x55,
                0x44,
                0x33,
                0x22,
                0x11,
                0b10_1000_00,
                0x99,
                0x88,
                0x77,
            ];

            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(
                iter.next(),
                Some(FifoData::TemperatureAndPressure(0x00445566, 0x00112233))
            );

            assert_eq!(iter.next(), Some(FifoData::Time(0x00778899)));
            assert_eq!(iter.next(), None);
        }

        #[test]
        fn empty_in_between() {
            let data = [
                0b10_0100_00,
                0x33,
                0x22,
                0x11,
                0b10_0000_00,
                0,
                0b10_1000_00,
                0x99,
                0x88,
                0x77,
            ];

            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::Temperature(0x00112233)));
            assert_eq!(iter.next(), None);
            assert_eq!(iter.next(), None, "the iterator should be fused");
        }

        #[test]
        fn configuration_error() {
            let data = [0b01_0001_00, 0x01, 0b10_0100_00, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::ConfigurationError));
            assert_eq!(iter.next(), Some(FifoData::Temperature(0x00112233)));
            assert_eq!(iter.next(), None);
        }

        #[test]
        fn fifo_input_configuration_change() {
            let data = [0b01_0010_00, 0x01, 0b10_0100_00, 0x33, 0x22, 0x11];
            let mut iter = FifoIter::new(data.into_iter());
            assert_eq!(iter.next(), Some(FifoData::FifoInputConfigurationChange));
            assert_eq!(iter.next(), Some(FifoData::Temperature(0x00112233)));
            assert_eq!(iter.next(), None);
        }
    }
}
