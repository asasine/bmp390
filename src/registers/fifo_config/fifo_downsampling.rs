use crate::registers::ReservedValueError;

/// How often output samples are written to the FIFO.
///
/// Downsampling affects only FIFO storage. The sensor continues measuring, updating the IIR
/// filter, and producing data-ready events at the configured output data rate. Therefore,
/// downsampling reduces FIFO usage and bus traffic, but does not reduce measurement activity or
/// power consumption.
///
/// When [`super::FifoDataSelect::Filtered`] is selected, the IIR filter runs at the output data
/// rate before its output is downsampled for the FIFO.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[doc(alias = "FifoSubsampling")]
#[doc(alias = "fifo_subsampling")]
pub enum FifoDownsampling {
    /// Store every sample.
    EverySample,

    /// Store every second sample.
    Every2,

    /// Store every fourth sample.
    Every4,

    /// Store every eighth sample.
    Every8,

    /// Store every sixteenth sample.
    Every16,

    /// Store every thirty-second sample.
    Every32,

    /// Store every sixty-fourth sample.
    Every64,

    /// Store every one-hundred-twenty-eighth sample.
    Every128,
}

impl FifoDownsampling {
    /// The register default: store every fourth sample.
    pub const DEFAULT: Self = Self::Every4;

    /// Return the register exponent used to encode the divisor.
    ///
    /// The effective FIFO rate divisor is `2^exponent`.
    pub const fn exponent(self) -> u8 {
        match self {
            Self::EverySample => 0,
            Self::Every2 => 1,
            Self::Every4 => 2,
            Self::Every8 => 3,
            Self::Every16 => 4,
            Self::Every32 => 5,
            Self::Every64 => 6,
            Self::Every128 => 7,
        }
    }

    /// Return the divisor applied to the output data rate for FIFO storage.
    pub const fn factor(self) -> u8 {
        1 << self.exponent()
    }
}

impl Default for FifoDownsampling {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl TryFrom<u8> for FifoDownsampling {
    type Error = ReservedValueError;

    /// Convert a raw register exponent without truncating invalid values.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::EverySample),
            1 => Ok(Self::Every2),
            2 => Ok(Self::Every4),
            3 => Ok(Self::Every8),
            4 => Ok(Self::Every16),
            5 => Ok(Self::Every32),
            6 => Ok(Self::Every64),
            7 => Ok(Self::Every128),
            value => Err(ReservedValueError::new(
                "fifo_config_2.fifo_subsampling",
                value,
            )),
        }
    }
}

impl From<FifoDownsampling> for u8 {
    fn from(value: FifoDownsampling) -> Self {
        value.exponent()
    }
}

impl FifoDownsampling {
    pub(super) fn from_register_exponent(value: u8) -> Self {
        match value & 0b111 {
            0 => Self::EverySample,
            1 => Self::Every2,
            2 => Self::Every4,
            3 => Self::Every8,
            4 => Self::Every16,
            5 => Self::Every32,
            6 => Self::Every64,
            _ => Self::Every128,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [FifoDownsampling; 8] = [
        FifoDownsampling::EverySample,
        FifoDownsampling::Every2,
        FifoDownsampling::Every4,
        FifoDownsampling::Every8,
        FifoDownsampling::Every16,
        FifoDownsampling::Every32,
        FifoDownsampling::Every64,
        FifoDownsampling::Every128,
    ];

    #[test]
    fn exponents_and_factors_match_variants() {
        for (exponent, downsampling) in ALL.into_iter().enumerate() {
            assert_eq!(downsampling.exponent(), exponent as u8);
            assert_eq!(downsampling.factor(), 1 << exponent);
        }
    }

    #[test]
    fn raw_exponents_roundtrip() {
        for downsampling in ALL {
            assert_eq!(
                FifoDownsampling::try_from(downsampling.exponent()).unwrap(),
                downsampling,
            );
        }
    }

    #[test]
    fn invalid_raw_exponents_are_rejected() {
        for value in 8..=u8::MAX {
            assert_eq!(
                FifoDownsampling::try_from(value),
                Err(ReservedValueError::new(
                    "fifo_config_2.fifo_subsampling",
                    value,
                )),
            );
        }
    }

    #[test]
    fn register_exponents_are_safely_masked_to_three_bits() {
        for value in 0..=u8::MAX {
            assert_eq!(
                FifoDownsampling::from_register_exponent(value).exponent(),
                value & 0b111,
            );
        }
    }
}
