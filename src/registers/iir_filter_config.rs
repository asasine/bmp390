use crate::raw;

/// Coefficient controlling the sensor's first-order IIR low-pass filter.
///
/// As defined in BMP390 datasheet section 3.4.3, for filter coefficient `c`:
/// `data_filtered = (data_filtered_old * c + data_ADC) / (c + 1)`.
/// Larger coefficients suppress more short-term noise but add more response latency.
/// [`Self::Coefficient0`] bypasses the filter.
///
/// The filter is updated at the configured output data rate. FIFO downsampling occurs afterward
/// when [`super::FifoDataSelect::Filtered`] is selected; it does not reduce the filter update rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FilterCoefficient {
    /// Bypass filtering (`c = 0`).
    Coefficient0,

    /// Light smoothing (`c = 1`).
    Coefficient1,

    /// Coefficient 3.
    Coefficient3,

    /// Coefficient 7.
    Coefficient7,

    /// Coefficient 15.
    Coefficient15,

    /// Coefficient 31.
    Coefficient31,

    /// Coefficient 63.
    Coefficient63,

    /// Coefficient 127.
    Coefficient127,
}

impl FilterCoefficient {
    /// The default filter coefficient: bypass mode.
    pub const DEFAULT: Self = Self::Coefficient0;

    /// Return the numeric coefficient used by the IIR filter.
    pub const fn coefficient(self) -> u8 {
        match self {
            Self::Coefficient0 => 0,
            Self::Coefficient1 => 1,
            Self::Coefficient3 => 3,
            Self::Coefficient7 => 7,
            Self::Coefficient15 => 15,
            Self::Coefficient31 => 31,
            Self::Coefficient63 => 63,
            Self::Coefficient127 => 127,
        }
    }
}

impl Default for FilterCoefficient {
    /// The default filter coefficient: bypass mode.
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl From<raw::IirFilterCoefficient> for FilterCoefficient {
    fn from(value: raw::IirFilterCoefficient) -> Self {
        match value {
            raw::IirFilterCoefficient::Coef0 => Self::Coefficient0,
            raw::IirFilterCoefficient::Coef1 => Self::Coefficient1,
            raw::IirFilterCoefficient::Coef3 => Self::Coefficient3,
            raw::IirFilterCoefficient::Coef7 => Self::Coefficient7,
            raw::IirFilterCoefficient::Coef15 => Self::Coefficient15,
            raw::IirFilterCoefficient::Coef31 => Self::Coefficient31,
            raw::IirFilterCoefficient::Coef63 => Self::Coefficient63,
            raw::IirFilterCoefficient::Coef127 => Self::Coefficient127,
        }
    }
}

impl From<FilterCoefficient> for raw::IirFilterCoefficient {
    fn from(value: FilterCoefficient) -> Self {
        match value {
            FilterCoefficient::Coefficient0 => Self::Coef0,
            FilterCoefficient::Coefficient1 => Self::Coef1,
            FilterCoefficient::Coefficient3 => Self::Coef3,
            FilterCoefficient::Coefficient7 => Self::Coef7,
            FilterCoefficient::Coefficient15 => Self::Coef15,
            FilterCoefficient::Coefficient31 => Self::Coef31,
            FilterCoefficient::Coefficient63 => Self::Coef63,
            FilterCoefficient::Coefficient127 => Self::Coef127,
        }
    }
}

impl From<FilterCoefficient> for raw::Config {
    fn from(value: FilterCoefficient) -> Self {
        let mut register = Self::default();
        register.set_iir_filter(value.into());
        register
    }
}

impl From<raw::Config> for FilterCoefficient {
    fn from(value: raw::Config) -> Self {
        value.iir_filter().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [FilterCoefficient; 8] = [
        FilterCoefficient::Coefficient0,
        FilterCoefficient::Coefficient1,
        FilterCoefficient::Coefficient3,
        FilterCoefficient::Coefficient7,
        FilterCoefficient::Coefficient15,
        FilterCoefficient::Coefficient31,
        FilterCoefficient::Coefficient63,
        FilterCoefficient::Coefficient127,
    ];

    #[test]
    fn filter_coefficient_roundtrips() {
        for value in ALL {
            assert_eq!(
                FilterCoefficient::from(raw::IirFilterCoefficient::from(value)),
                value
            );
        }
    }

    #[test]
    fn register_roundtrips() {
        for coefficient in ALL {
            assert_eq!(
                FilterCoefficient::from(raw::Config::from(coefficient)),
                coefficient,
            );
        }
    }

    #[test]
    fn numeric_coefficients_match_variants() {
        assert_eq!(
            ALL.map(FilterCoefficient::coefficient),
            [0, 1, 3, 7, 15, 31, 63, 127],
        );
    }
}
