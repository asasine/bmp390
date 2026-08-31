use super::ReservedValueError;
use crate::raw::{self, field_sets};

/// Command accepted by the BMP390 command register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Clear all FIFO data.
    FlushFifo,

    /// Reset the device and its user configuration.
    SoftReset,
}

impl TryFrom<raw::Command> for Command {
    type Error = ReservedValueError;

    fn try_from(value: raw::Command) -> Result<Self, Self::Error> {
        match value {
            raw::Command::FifoFlush => Ok(Self::FlushFifo),
            raw::Command::SoftReset => Ok(Self::SoftReset),
            raw::Command::Nop => Err(ReservedValueError::new("cmd.cmd", 0)),
            raw::Command::Reserved(value) => Err(ReservedValueError::new("cmd.cmd", value)),
        }
    }
}

impl From<Command> for raw::Command {
    fn from(value: Command) -> Self {
        match value {
            Command::FlushFifo => Self::FifoFlush,
            Command::SoftReset => Self::SoftReset,
        }
    }
}

/// Command register value represented by [`field_sets::CmdFieldsIn`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CommandRegister {
    /// Command to execute.
    pub command: Command,
}

impl TryFrom<field_sets::CmdFieldsIn> for Command {
    type Error = ReservedValueError;

    fn try_from(value: field_sets::CmdFieldsIn) -> Result<Self, Self::Error> {
        value.cmd().try_into()
    }
}

impl From<Command> for field_sets::CmdFieldsIn {
    fn from(value: Command) -> Self {
        let mut register = Self::new_zero();
        register.set_cmd(value.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Command; 2] = [Command::FlushFifo, Command::SoftReset];

    #[test]
    fn command_roundtrips() {
        for value in ALL {
            assert_eq!(Command::try_from(raw::Command::from(value)), Ok(value));
        }
    }

    #[test]
    fn roundtrips() {
        for command in ALL {
            assert_eq!(
                Command::try_from(field_sets::CmdFieldsIn::from(command)),
                Ok(command)
            );
        }
    }

    #[test]
    fn error_reports_field() {
        let mut command = field_sets::CmdFieldsIn::new_zero();
        command.set_cmd(raw::Command::Reserved(1));
        assert_eq!(
            Command::try_from(command),
            Err(ReservedValueError::new("cmd.cmd", 1))
        );
    }
}
