use crate::raw::field_sets;

/// Sensor error conditions from [`field_sets::ErrReg`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ErrorStatus {
    /// Whether the sensor has encountered a fatal error.
    pub fatal: bool,

    /// Whether command execution failed. Cleared on read.
    pub command: bool,

    /// Whether the sensor configuration is invalid. Cleared on read.
    pub configuration: bool,
}

impl From<field_sets::ErrReg> for ErrorStatus {
    fn from(value: field_sets::ErrReg) -> Self {
        Self {
            fatal: value.fatal_err(),
            command: value.cmd_err(),
            configuration: value.conf_err(),
        }
    }
}

impl From<ErrorStatus> for field_sets::ErrReg {
    fn from(value: ErrorStatus) -> Self {
        let mut register = Self::new_zero();
        register.set_fatal_err(value.fatal);
        register.set_cmd_err(value.command);
        register.set_conf_err(value.configuration);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        let errors = ErrorStatus {
            fatal: true,
            command: false,
            configuration: true,
        };

        assert_eq!(ErrorStatus::from(field_sets::ErrReg::from(errors)), errors);
    }
}
