mod fifo_data_select;
mod fifo_subsampling;
mod fifo_watermark;

pub use fifo_data_select::FifoDataSelect;
pub use fifo_subsampling::FifoSubsampling;
pub use fifo_watermark::FifoWatermark;

use crate::raw;

/// FIFO mode selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// FIFO is disabled.
    Disabled,

    /// FIFO is enabled.
    Enabled,
}

impl From<raw::FifoMode> for FifoMode {
    fn from(value: raw::FifoMode) -> Self {
        match value {
            raw::FifoMode::Disabled => Self::Disabled,
            raw::FifoMode::Enabled => Self::Enabled,
        }
    }
}

impl From<FifoMode> for raw::FifoMode {
    fn from(value: FifoMode) -> Self {
        match value {
            FifoMode::Disabled => Self::Disabled,
            FifoMode::Enabled => Self::Enabled,
        }
    }
}

/// FIFO configuration represented by [`FifoConfig`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoConfig {
    /// Whether the FIFO is enabled.
    pub mode: FifoMode,

    /// Stop writing when the FIFO is full.
    pub stop_on_full: bool,

    /// Include sensor time in the FIFO stream.
    pub time_enable: bool,

    /// Store pressure data in the FIFO.
    pub pressure_enable: bool,

    /// Store temperature data in the FIFO.
    pub temperature_enable: bool,

    /// FIFO downsampling selection. The effective period is `2^subsampling`.
    pub subsampling: FifoSubsampling,

    /// Data source used for FIFO samples.
    pub data_select: FifoDataSelect,
}

impl FifoConfig {
    /// The default FIFO configuration.
    ///
    /// - FIFO is disabled.
    /// - Stop on full is enabled.
    /// - Time, pressure, and temperature are disabled.
    /// - Subsampling is set to 0.
    /// - Filtered data source.
    pub const DEFAULT: Self = Self {
        mode: FifoMode::Disabled,
        stop_on_full: true,
        time_enable: false,
        pressure_enable: false,
        temperature_enable: false,
        subsampling: FifoSubsampling::try_new(0).unwrap(),
        data_select: FifoDataSelect::Filtered,
    };
}

impl Default for FifoConfig {
    /// The default FIFO configuration.
    ///
    /// - FIFO is disabled.
    /// - Stop on full is enabled.
    /// - Time, pressure, and temperature are disabled.
    /// - Subsampling is set to 0.
    /// - Filtered data source.
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl From<raw::FifoConfig> for FifoConfig {
    fn from(value: raw::FifoConfig) -> Self {
        Self {
            mode: value.mode().into(),
            stop_on_full: value.stop_on_full(),
            time_enable: value.time_enable(),
            pressure_enable: value.pressure_enable(),
            temperature_enable: value.temperature_enable(),
            subsampling: value.subsampling().into(),
            data_select: value.data_select().into(),
        }
    }
}

impl From<FifoConfig> for raw::FifoConfig {
    fn from(value: FifoConfig) -> Self {
        let mut register = Self::default();
        register.set_mode(value.mode.into());
        register.set_stop_on_full(value.stop_on_full);
        register.set_time_enable(value.time_enable);
        register.set_pressure_enable(value.pressure_enable);
        register.set_temperature_enable(value.temperature_enable);
        register.set_subsampling(value.subsampling.value());
        register.set_data_select(value.data_select.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_mode_roundtrips() {
        for value in [FifoMode::Disabled, FifoMode::Enabled] {
            assert_eq!(FifoMode::from(raw::FifoMode::from(value)), value);
        }
    }

    #[test]
    fn roundtrips() {
        let fifo = FifoConfig {
            mode: FifoMode::Enabled,
            stop_on_full: true,
            time_enable: false,
            pressure_enable: true,
            temperature_enable: true,
            subsampling: FifoSubsampling::try_new(2).unwrap(),
            data_select: FifoDataSelect::Filtered,
        };

        assert_eq!(FifoConfig::from(FifoConfig::from(fifo)), fifo);
    }
}
