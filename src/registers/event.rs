use crate::raw::field_sets;

/// Event flags from [`field_sets::Event`]. These flags are cleared on read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Event {
    /// Whether a power-on reset was detected.
    pub power_on_reset: bool,

    /// Whether an interface transaction occurred during a conversion.
    pub interface_activity: bool,
}

impl From<field_sets::Event> for Event {
    fn from(value: field_sets::Event) -> Self {
        Self {
            power_on_reset: value.por_detected(),
            interface_activity: value.itf_act_pt(),
        }
    }
}

impl From<Event> for field_sets::Event {
    fn from(value: Event) -> Self {
        let mut register = Self::new_zero();
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

        assert_eq!(Event::from(field_sets::Event::from(event)), event);
    }
}
