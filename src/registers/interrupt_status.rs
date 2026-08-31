use crate::raw::field_sets;

/// Interrupt flags from [`field_sets::IntStatus`]. These flags are cleared on read.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptStatus {
    /// Whether the FIFO watermark interrupt is active.
    pub fifo_watermark: bool,

    /// Whether the FIFO full interrupt is active.
    pub fifo_full: bool,

    /// Whether the data-ready interrupt is active.
    pub data_ready: bool,
}

impl From<field_sets::IntStatus> for InterruptStatus {
    fn from(value: field_sets::IntStatus) -> Self {
        Self {
            fifo_watermark: value.fifo_full_watermark(),
            fifo_full: value.fifo_full(),
            data_ready: value.data_ready(),
        }
    }
}

impl From<InterruptStatus> for field_sets::IntStatus {
    fn from(value: InterruptStatus) -> Self {
        let mut register = Self::new_zero();
        register.set_fifo_full_watermark(value.fifo_watermark);
        register.set_fifo_full(value.fifo_full);
        register.set_data_ready(value.data_ready);
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        let interrupt_status = InterruptStatus {
            fifo_watermark: true,
            fifo_full: false,
            data_ready: true,
        };

        assert_eq!(
            InterruptStatus::from(field_sets::IntStatus::from(interrupt_status)),
            interrupt_status
        );
    }
}
