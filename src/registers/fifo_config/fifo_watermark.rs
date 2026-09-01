use crate::raw;

/// FIFO watermark level.
///
/// Since the FIFO watermark level is represented by a 9-bit value, the maximum valid level is
/// `0x1FF` ([`Self::MAX`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoWatermark(u16);

impl FifoWatermark {
    /// The default FIFO watermark level (`0x01`, one sample).
    pub const DEFAULT: Self = Self(0x01);

    /// Maximum valid FIFO watermark level.
    pub const MAX: Self = Self(0x01ff);

    /// Create a new FIFO watermark level if it is valid for the hardware.
    ///
    /// The provided level must be less than or equal to [`Self::MAX`].
    pub const fn try_new(level: u16) -> Option<Self> {
        if level <= Self::MAX.0 {
            Some(Self(level))
        } else {
            None
        }
    }

    /// Create a new FIFO watermark level, masking any bits beyond the maximum valid level.
    ///
    /// Compared with [`Self::try_new`], this method will always return a valid FIFO watermark level
    /// by masking any excess bits. This may result in a different [`Self::level`] than was provided.
    ///
    /// ```rust
    /// # use bmp390::registers::FifoWatermark;
    /// let wm = FifoWatermark::new_masked(0xFFF);
    /// assert_eq!(wm.level(), 0x1FF); // Excess bits are masked
    /// ```
    pub const fn new_masked(level: u16) -> Self {
        Self(level & 0x1FF)
    }

    /// Get the FIFO watermark level, in sensor samples.
    pub const fn level(&self) -> u16 {
        self.0
    }
}

impl Default for FifoWatermark {
    /// The default FIFO watermark level (`0x01`, one sample).
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl From<raw::FifoWatermark> for FifoWatermark {
    /// Convert a [`raw::FifoWatermark`] into a [`FifoWatermark`] using [`Self::new_masked`].
    fn from(value: raw::FifoWatermark) -> Self {
        let raw = u16::from_le_bytes(value.into());
        Self::new_masked(raw)
    }
}

impl From<FifoWatermark> for raw::FifoWatermark {
    fn from(value: FifoWatermark) -> Self {
        value.0.to_le_bytes().into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_new_lte_max_is_valid() {
        for value in 0..=FifoWatermark::MAX.0 {
            let wm = FifoWatermark::try_new(value).expect("Value less than MAX should be valid");
            assert_eq!(wm.level(), value);
        }
    }

    #[test]
    fn try_new_gt_max_is_invalid() {
        for value in (FifoWatermark::MAX.0 + 1)..=u16::MAX {
            let wm = FifoWatermark::try_new(value);
            assert!(wm.is_none());
        }
    }

    #[test]
    fn new_masked_masks_excess_bits() {
        for value in 0..=u16::MAX {
            let wm = FifoWatermark::new_masked(value);
            assert_eq!(wm.level(), value & 0x1FF); // Excess bits are masked
        }
    }

    #[test]
    fn from_raw_masks_excess_bits() {
        // 0xFFFF exceeds 9-bit maximum
        let raw = raw::FifoWatermark::from([0xFF, 0xFF]);
        let fifo_watermark = FifoWatermark::from(raw);
        assert_eq!(fifo_watermark.0, 0x1FF); // Only lower 9 bits should be kept
    }
}
