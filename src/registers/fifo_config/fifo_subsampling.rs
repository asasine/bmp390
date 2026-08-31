/// FIFO downsampling selection for pressure and temperature data. Factor is `2^fifo_subsampling`.
///
/// Since a [`FifoSubsampling`] has a maximum value, there are three ways to create an instance:
/// 1. [`try_new`] - Returns [`None`] if the value is too large.
/// 2. [`new_masked`] - Masks the value's bits to fit within the valid range.
/// 3. [`new_clamped`] - Clamps the value to the valid range.
///
/// [`try_new`]: Self::try_new
/// [`new_masked`]: Self::new_masked
/// [`new_clamped`]: Self::new_clamped
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoSubsampling(u8);

impl FifoSubsampling {
    /// The maximum valid FIFO subsampling value.
    ///
    /// This corresponds to a subsampling factor of `2^7`.
    pub const MAX: Self = Self(0b111);

    /// Create a new FIFO subsampling configuration if it is valid.
    ///
    /// The provided subsampling factor must be less than or equal to [`Self::MAX`].
    pub const fn try_new(value: u8) -> Option<Self> {
        if value <= Self::MAX.0 {
            Some(Self(value))
        } else {
            None
        }
    }

    /// Convert a raw subsampling value into a [`FifoSubsampling`], masking out invalid bits.
    ///
    /// Compared to [`Self::try_new`], this function will always return a valid [`FifoSubsampling`],
    /// but the `value` will be masked to fit within the valid range, so [`Self::value`] may differ
    /// from the original `value`.
    ///
    /// ```rust
    /// # use bmp390::registers::FifoSubsampling;
    /// let subsampling = FifoSubsampling::new_masked(0xF);
    /// assert_eq!(subsampling.value(), 0x7); // 0xF (15) masked to fit into 3 bits.
    /// ```
    ///
    /// Use [`Self::new_clamped`] if you want to ensure the value is within the valid range without
    /// masking.
    pub const fn new_masked(value: u8) -> Self {
        Self(value & Self::MAX.0)
    }

    /// Create a new FIFO subsampling configuration, clamping the value to the valid range.
    ///
    /// Compared to [`Self::try_new`], this function will always return a valid [`FifoSubsampling`],
    /// but the `value` will be clamped to fit within the valid range, so [`Self::value`] may differ
    /// from the original `value`.
    ///
    /// ```rust
    /// # use bmp390::registers::FifoSubsampling;
    /// let subsampling = FifoSubsampling::new_clamped(10);
    /// assert_eq!(subsampling.value(), 7); // 10 clamped to the max value
    /// ```
    pub const fn new_clamped(value: u8) -> Self {
        if value > Self::MAX.0 {
            Self::MAX
        } else {
            Self(value)
        }
    }

    /// Get the raw subsampling value.
    ///
    /// The subsampling factor is `2^value`.
    pub const fn value(&self) -> u8 {
        self.0
    }

    /// Get the effective subsampling factor.
    ///
    /// This is `2^value`.
    pub const fn factor(&self) -> u8 {
        1 << self.0
    }
}

impl From<u8> for FifoSubsampling {
    /// Convert a raw [`u8`] value into a [`FifoSubsampling`] using [`Self::new_masked`].
    fn from(value: u8) -> Self {
        FifoSubsampling::new_masked(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_subsampling_try_new() {
        for value in 0..=FifoSubsampling::MAX.value() {
            let subsampling = FifoSubsampling::try_new(value);
            assert_eq!(subsampling.unwrap().value(), value);
        }

        for value in 8..=u8::MAX {
            let subsampling = FifoSubsampling::try_new(value);
            assert!(subsampling.is_none());
        }
    }

    #[test]
    fn fifo_subsampling_new_masked() {
        for value in 0..=u8::MAX {
            let subsampling = FifoSubsampling::new_masked(value);
            assert_eq!(subsampling.value(), value & 0b111); // Only lower 3 bits should be kept
        }
    }

    #[test]
    fn fifo_subsampling_new_clamped() {
        for value in FifoSubsampling::MAX.value()..=u8::MAX {
            let subsampling = FifoSubsampling::new_clamped(value);
            assert_eq!(subsampling, FifoSubsampling::MAX);
        }
    }

    #[test]
    fn from_u8_masks_excess_bits() {
        for value in 0..=u8::MAX {
            let subsampling = FifoSubsampling::from(value);
            assert_eq!(subsampling.value(), value & 0b111); // Only lower 3 bits should be kept
        }
    }
}
