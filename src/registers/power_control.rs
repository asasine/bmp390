use crate::raw::{self, field_sets};

/// Sensor measurement mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PowerMode {
    /// No measurements are performed.
    Sleep,

    /// One forced measurement is performed before returning to sleep.
    Forced,

    /// Measurements are performed continuously at the configured rate.
    Normal,
}

impl From<raw::MeasurementMode> for PowerMode {
    fn from(value: raw::MeasurementMode) -> Self {
        match value {
            raw::MeasurementMode::Sleep => Self::Sleep,
            raw::MeasurementMode::Forced => Self::Forced,
            raw::MeasurementMode::Forced2 => Self::Forced,
            raw::MeasurementMode::Normal => Self::Normal,
        }
    }
}

impl From<PowerMode> for raw::MeasurementMode {
    fn from(value: PowerMode) -> Self {
        match value {
            PowerMode::Sleep => Self::Sleep,
            PowerMode::Forced => Self::Forced,
            PowerMode::Normal => Self::Normal,
        }
    }
}

/// Power and measurement configuration represented by [`field_sets::PwrCtrl`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerControl {
    /// Whether pressure measurements are enabled.
    pub pressure_enabled: bool,

    /// Whether temperature measurements are enabled.
    pub temperature_enabled: bool,

    /// Sensor measurement mode.
    pub mode: PowerMode,
}

impl From<field_sets::PwrCtrl> for PowerControl {
    fn from(value: field_sets::PwrCtrl) -> Self {
        Self {
            pressure_enabled: value.pressure_enable(),
            temperature_enabled: value.temperature_enable(),
            mode: value.mode().into(),
        }
    }
}

impl From<PowerControl> for field_sets::PwrCtrl {
    fn from(value: PowerControl) -> Self {
        let mut register = Self::new_zero();
        register.set_pressure_enable(value.pressure_enabled);
        register.set_temperature_enable(value.temperature_enabled);
        register.set_mode(value.mode.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn power_mode_roundtrips() {
        for value in [PowerMode::Sleep, PowerMode::Forced, PowerMode::Normal] {
            assert_eq!(PowerMode::from(raw::MeasurementMode::from(value)), value);
        }
    }

    #[test]
    fn roundtrips() {
        let power = PowerControl {
            pressure_enabled: true,
            temperature_enabled: false,
            mode: PowerMode::Forced,
        };
        assert_eq!(PowerControl::from(field_sets::PwrCtrl::from(power)), power);
    }
}
