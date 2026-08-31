use crate::raw::{self, field_sets};

/// Sensor status flags from [`field_sets::Status`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Whether the command decoder can accept a command.
    pub command_ready: bool,

    /// Whether new pressure data is available.
    pub pressure_ready: bool,

    /// Whether new temperature data is available.
    pub temperature_ready: bool,
}

impl From<field_sets::Status> for Status {
    fn from(value: field_sets::Status) -> Self {
        Self {
            command_ready: value.command_ready() == raw::CommandStatus::Ready,
            pressure_ready: value.data_ready_pressure(),
            temperature_ready: value.data_ready_temperature(),
        }
    }
}

impl From<Status> for field_sets::Status {
    fn from(value: Status) -> Self {
        let mut register = Self::new_zero();
        register.set_command_ready(if value.command_ready {
            raw::CommandStatus::Ready
        } else {
            raw::CommandStatus::InProgress
        });

        register.set_data_ready_pressure(value.pressure_ready);
        register.set_data_ready_temperature(value.temperature_ready);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_round_trip() {
        let status = Status {
            command_ready: true,
            pressure_ready: false,
            temperature_ready: true,
        };

        assert_eq!(Status::from(field_sets::Status::from(status)), status);
    }
}
