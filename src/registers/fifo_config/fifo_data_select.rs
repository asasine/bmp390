use crate::raw;

/// FIFO data source selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoDataSelect {
    /// Write unfiltered measurements to the FIFO.
    ///
    /// The configured IIR filter does not affect FIFO samples in this mode.
    Unfiltered,

    /// Write IIR-filtered measurements to the FIFO.
    ///
    /// The IIR filter is updated at the sensor output data rate before FIFO downsampling is
    /// applied.
    Filtered,
}

impl From<raw::FifoDataSelect> for FifoDataSelect {
    /// Convert a [`raw::FifoDataSelect`] into a [`FifoDataSelect`].
    ///
    /// Reserved values are mapped to [`FifoDataSelect::Unfiltered`].
    fn from(value: raw::FifoDataSelect) -> Self {
        match value {
            raw::FifoDataSelect::Unfiltered => Self::Unfiltered,
            raw::FifoDataSelect::Filtered => Self::Filtered,
            raw::FifoDataSelect::Reserved(_) => Self::Unfiltered,
        }
    }
}

impl From<FifoDataSelect> for raw::FifoDataSelect {
    fn from(value: FifoDataSelect) -> Self {
        match value {
            FifoDataSelect::Unfiltered => Self::Unfiltered,
            FifoDataSelect::Filtered => Self::Filtered,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        for value in [FifoDataSelect::Unfiltered, FifoDataSelect::Filtered] {
            assert_eq!(
                FifoDataSelect::from(raw::FifoDataSelect::from(value)),
                value,
            );
        }
    }

    #[test]
    fn reserved_values_map_to_unfiltered() {
        let raw = (0..=u8::MAX)
            .map(raw::FifoDataSelect::from)
            .filter(|value| matches!(value, raw::FifoDataSelect::Reserved(_)));

        for value in raw {
            let parsed = FifoDataSelect::from(value);
            assert_eq!(parsed, FifoDataSelect::Unfiltered);
        }
    }
}
