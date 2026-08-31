use crate::raw::{self, field_sets};

/// Command execution status.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandStatus {
    /// Command execution in progress.
    InProgress = 0,

    /// Command decoder is ready to accept a new command.
    Ready = 1,
}

impl From<raw::CommandStatus> for CommandStatus {
    fn from(value: raw::CommandStatus) -> Self {
        match value {
            raw::CommandStatus::Ready => CommandStatus::Ready,
            raw::CommandStatus::InProgress => CommandStatus::InProgress,
        }
    }
}

impl From<CommandStatus> for raw::CommandStatus {
    fn from(value: CommandStatus) -> Self {
        match value {
            CommandStatus::Ready => raw::CommandStatus::Ready,
            CommandStatus::InProgress => raw::CommandStatus::InProgress,
        }
    }
}

/// Sensor status flags from [`field_sets::Status`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Whether the command decoder can accept a command.
    pub command_ready: CommandStatus,

    /// Whether new pressure data is available.
    ///
    /// Cleared when one pressure measurement is read.
    pub pressure_ready: bool,

    /// Whether new temperature data is available.
    ///
    /// Cleared when one temperature measurement is read.
    pub temperature_ready: bool,
}

impl From<field_sets::Status> for Status {
    fn from(value: field_sets::Status) -> Self {
        Self {
            command_ready: value.command_ready().into(),
            pressure_ready: value.data_ready_pressure(),
            temperature_ready: value.data_ready_temperature(),
        }
    }
}

impl From<Status> for field_sets::Status {
    fn from(value: Status) -> Self {
        let mut register = Self::new_zero();
        register.set_command_ready(value.command_ready.into());
        register.set_data_ready_pressure(value.pressure_ready);
        register.set_data_ready_temperature(value.temperature_ready);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [CommandStatus; 2] = [CommandStatus::InProgress, CommandStatus::Ready];

    #[test]
    fn command_status_roundtrips() {
        for status in ALL {
            assert_eq!(CommandStatus::from(raw::CommandStatus::from(status)), status);
        }
    }

    #[test]
    fn roundtrips() {
        for status in ALL {
            let status = Status {
                command_ready: status,
                pressure_ready: false,
                temperature_ready: true,
            };

            assert_eq!(Status::from(field_sets::Status::from(status)), status);
        }
    }
}
