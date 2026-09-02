mod fifo_data_select;
mod fifo_downsampling;
mod fifo_watermark;

pub use fifo_data_select::FifoDataSelect;
pub use fifo_downsampling::FifoDownsampling;
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

/// FIFO configuration.
///
/// FIFO downsampling is independent of the sensor's output data rate (ODR). For example, an ODR
/// of 50 Hz and [`FifoDownsampling::Every8`] stores FIFO samples at 6.25 Hz while measurements and
/// data-ready events continue at 50 Hz. If [`FifoDataSelect::Filtered`] is selected, the IIR filter
/// is updated at 50 Hz and its output is then downsampled for FIFO storage.
///
/// ```
/// use bmp390::{
///     ConfigurationBuilder,
///     registers::{
///         FifoConfig, FifoDataSelect, FifoDownsampling, FilterCoefficient, OutputDataRate,
///     },
/// };
///
/// let configuration = ConfigurationBuilder::new()
///     .output_data_rate(OutputDataRate::Hz50)
///     .iir_filter(FilterCoefficient::Coefficient15)
///     .fifo_config(FifoConfig {
///         downsampling: FifoDownsampling::Every8,
///         data_select: FifoDataSelect::Filtered,
///         ..FifoConfig::default()
///     });
/// ```
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

    /// How often output samples are written to the FIFO.
    ///
    /// This does not change the measurement rate or reduce sensor power consumption.
    pub downsampling: FifoDownsampling,

    /// Data source used for FIFO samples.
    pub data_select: FifoDataSelect,
}

impl FifoConfig {
    /// The default FIFO configuration.
    ///
    /// - FIFO is disabled.
    /// - Stop on full is enabled.
    /// - Time, pressure, and temperature are disabled.
    /// - Every fourth sample is stored.
    /// - Unfiltered data source.
    pub const DEFAULT: Self = Self {
        mode: FifoMode::Disabled,
        stop_on_full: true,
        time_enable: false,
        pressure_enable: false,
        temperature_enable: false,
        downsampling: FifoDownsampling::DEFAULT,
        data_select: FifoDataSelect::Unfiltered,
    };
}

impl Default for FifoConfig {
    /// The default FIFO configuration.
    ///
    /// - FIFO is disabled.
    /// - Stop on full is enabled.
    /// - Time, pressure, and temperature are disabled.
    /// - Every fourth sample is stored.
    /// - Unfiltered data source.
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
            downsampling: FifoDownsampling::from_register_exponent(value.subsampling()),
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
        register.set_subsampling(value.downsampling.exponent());
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
            downsampling: FifoDownsampling::Every4,
            data_select: FifoDataSelect::Filtered,
        };

        assert_eq!(FifoConfig::from(raw::FifoConfig::from(fifo)), fifo);
    }

    #[test]
    fn default_matches_register_reset_value() {
        assert_eq!(
            FifoConfig::from(raw::FifoConfig::from([0x02, 0x02])),
            FifoConfig::DEFAULT
        );
    }
}
