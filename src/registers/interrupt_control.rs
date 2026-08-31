use crate::raw::{self, field_sets};

/// Electrical output mode for the interrupt pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptOutput {
    /// Push-pull output.
    PushPull,

    /// Open-drain output.
    OpenDrain,
}

impl From<raw::IntOpenDrain> for InterruptOutput {
    fn from(value: raw::IntOpenDrain) -> Self {
        match value {
            raw::IntOpenDrain::PushPull => Self::PushPull,
            raw::IntOpenDrain::OpenDrain => Self::OpenDrain,
        }
    }
}

impl From<InterruptOutput> for raw::IntOpenDrain {
    fn from(value: InterruptOutput) -> Self {
        match value {
            InterruptOutput::PushPull => Self::PushPull,
            InterruptOutput::OpenDrain => Self::OpenDrain,
        }
    }
}

/// Active level for the interrupt pin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptLevel {
    /// Active low.
    ActiveLow,

    /// Active high.
    ActiveHigh,
}

impl From<raw::IntLevel> for InterruptLevel {
    fn from(value: raw::IntLevel) -> Self {
        match value {
            raw::IntLevel::ActiveLow => Self::ActiveLow,
            raw::IntLevel::ActiveHigh => Self::ActiveHigh,
        }
    }
}

impl From<InterruptLevel> for raw::IntLevel {
    fn from(value: InterruptLevel) -> Self {
        match value {
            InterruptLevel::ActiveLow => Self::ActiveLow,
            InterruptLevel::ActiveHigh => Self::ActiveHigh,
        }
    }
}

/// Interrupt configuration represented by [`field_sets::IntCtrl`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptControl {
    /// Electrical output mode for the interrupt pin.
    pub output: InterruptOutput,

    /// Active level for the interrupt pin.
    pub level: InterruptLevel,

    /// Whether the interrupt remains latched until cleared.
    pub latched: bool,

    /// Whether the FIFO watermark interrupt is enabled.
    pub fifo_watermark_enabled: bool,

    /// Whether the FIFO full interrupt is enabled.
    pub fifo_full_enabled: bool,

    /// Interrupt drive-strength selection.
    pub drive_strength: bool,

    /// Whether the pressure and temperature data-ready interrupt is enabled.
    pub data_ready_enabled: bool,
}

impl From<field_sets::IntCtrl> for InterruptControl {
    fn from(value: field_sets::IntCtrl) -> Self {
        Self {
            output: value.int_open_drain().into(),
            level: value.int_level().into(),
            latched: value.int_latch(),
            fifo_watermark_enabled: value.fifo_watermark_int_enable(),
            fifo_full_enabled: value.fifo_full_int_enable(),
            drive_strength: value.int_ds(),
            data_ready_enabled: value.data_ready_int_enable(),
        }
    }
}

impl From<InterruptControl> for field_sets::IntCtrl {
    fn from(value: InterruptControl) -> Self {
        let mut register = Self::new_zero();
        register.set_int_open_drain(value.output.into());
        register.set_int_level(value.level.into());
        register.set_int_latch(value.latched);
        register.set_fifo_watermark_int_enable(value.fifo_watermark_enabled);
        register.set_fifo_full_int_enable(value.fifo_full_enabled);
        register.set_int_ds(value.drive_strength);
        register.set_data_ready_int_enable(value.data_ready_enabled);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interrupt_output_roundtrips() {
        for value in [InterruptOutput::PushPull, InterruptOutput::OpenDrain] {
            assert_eq!(InterruptOutput::from(raw::IntOpenDrain::from(value)), value);
        }
    }

    #[test]
    fn interrupt_level_roundtrips() {
        for value in [InterruptLevel::ActiveLow, InterruptLevel::ActiveHigh] {
            assert_eq!(InterruptLevel::from(raw::IntLevel::from(value)), value);
        }
    }

    #[test]
    fn roundtrips() {
        let interrupt_control = InterruptControl {
            output: InterruptOutput::OpenDrain,
            level: InterruptLevel::ActiveLow,
            latched: true,
            fifo_watermark_enabled: false,
            fifo_full_enabled: true,
            drive_strength: false,
            data_ready_enabled: true,
        };

        assert_eq!(
            InterruptControl::from(field_sets::IntCtrl::from(interrupt_control)),
            interrupt_control
        );
    }
}
