use crate::raw;

/// Event flags from [`Event`]. These flags are cleared on read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Event {
    /// Whether a power-on reset was detected.
    pub power_on_reset: bool,

    /// Whether an interface transaction occurred during a conversion.
    pub interface_activity: bool,
}

impl From<raw::Event> for Event {
    fn from(value: raw::Event) -> Self {
        Self {
            power_on_reset: value.por_detected(),
            interface_activity: value.itf_act_pt(),
        }
    }
}

impl From<Event> for raw::Event {
    fn from(value: Event) -> Self {
        let mut register = Self::default();
        register.set_por_detected(value.power_on_reset);
        register.set_itf_act_pt(value.interface_activity);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        let event = Event {
            power_on_reset: false,
            interface_activity: true,
        };

        assert_eq!(Event::from(raw::Event::from(event)), event);
    }
}
