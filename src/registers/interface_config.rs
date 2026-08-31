use crate::raw::{self, field_sets};

/// SPI mode selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    /// SPI 4-wire mode.
    Spi4,

    /// SPI 3-wire mode.
    Spi3,
}

impl From<raw::SpiMode> for SpiMode {
    fn from(value: raw::SpiMode) -> Self {
        match value {
            raw::SpiMode::Spi4 => Self::Spi4,
            raw::SpiMode::Spi3 => Self::Spi3,
        }
    }
}

impl From<SpiMode> for raw::SpiMode {
    fn from(value: SpiMode) -> Self {
        match value {
            SpiMode::Spi4 => Self::Spi4,
            SpiMode::Spi3 => Self::Spi3,
        }
    }
}

/// I2C watchdog timer period.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cWatchdogPeriod {
    /// Timeout after 1.25 ms.
    Short,

    /// Timeout after 40 ms.
    Long,
}

impl From<raw::I2CWatchdogPeriod> for I2cWatchdogPeriod {
    fn from(value: raw::I2CWatchdogPeriod) -> Self {
        match value {
            raw::I2CWatchdogPeriod::Short => Self::Short,
            raw::I2CWatchdogPeriod::Long => Self::Long,
        }
    }
}

impl From<I2cWatchdogPeriod> for raw::I2CWatchdogPeriod {
    fn from(value: I2cWatchdogPeriod) -> Self {
        match value {
            I2cWatchdogPeriod::Short => Self::Short,
            I2cWatchdogPeriod::Long => Self::Long,
        }
    }
}

/// Serial interface configuration represented by [`field_sets::IfConf`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterfaceConfig {
    /// SPI mode for the primary interface.
    pub spi: SpiMode,

    /// Whether the I2C watchdog is enabled.
    pub i2c_watchdog_enabled: bool,

    /// Timeout period for the I2C watchdog.
    pub i2c_watchdog_period: I2cWatchdogPeriod,
}

impl InterfaceConfig {
    /// The default interface configuration.
    ///
    /// - SPI 4-wire mode.
    /// - I2C watchdog enabled.
    /// - I2C watchdog period: short (1.25 ms).
    pub const DEFAULT: Self = Self {
        spi: SpiMode::Spi4,
        i2c_watchdog_enabled: true,
        i2c_watchdog_period: I2cWatchdogPeriod::Short,
    };
}

impl Default for InterfaceConfig {
    /// The default interface configuration.
    ///
    /// - SPI 4-wire mode.
    /// - I2C watchdog enabled.
    /// - I2C watchdog period: short (1.25 ms).
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl From<field_sets::IfConf> for InterfaceConfig {
    fn from(value: field_sets::IfConf) -> Self {
        Self {
            spi: value.spi().into(),
            i2c_watchdog_enabled: value.i_2_c_wdt_en(),
            i2c_watchdog_period: value.i_2_c_wdt_sel().into(),
        }
    }
}

impl From<InterfaceConfig> for field_sets::IfConf {
    fn from(value: InterfaceConfig) -> Self {
        let mut register = Self::new_zero();
        register.set_spi(value.spi.into());
        register.set_i_2_c_wdt_en(value.i2c_watchdog_enabled);
        register.set_i_2_c_wdt_sel(value.i2c_watchdog_period.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spi_mode_roundtrips() {
        for value in [SpiMode::Spi4, SpiMode::Spi3] {
            assert_eq!(SpiMode::from(raw::SpiMode::from(value)), value);
        }
    }

    #[test]
    fn i2c_watchdog_period_roundtrips() {
        for value in [I2cWatchdogPeriod::Short, I2cWatchdogPeriod::Long] {
            assert_eq!(
                I2cWatchdogPeriod::from(raw::I2CWatchdogPeriod::from(value)),
                value
            );
        }
    }

    #[test]
    fn roundtrips() {
        let interface = InterfaceConfig {
            spi: SpiMode::Spi3,
            i2c_watchdog_enabled: false,
            i2c_watchdog_period: I2cWatchdogPeriod::Short,
        };
        assert_eq!(
            InterfaceConfig::from(field_sets::IfConf::from(interface)),
            interface
        );
    }
}
