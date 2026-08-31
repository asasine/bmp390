use super::ReservedValueError;
use crate::raw::{self, field_sets};

/// Oversampling applied to a measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Oversampling {
    /// No oversampling.
    X1,

    /// 2x oversampling.
    X2,

    /// 4x oversampling.
    X4,

    /// 8x oversampling.
    X8,

    /// 16x oversampling.
    X16,

    /// 32x oversampling.
    X32,
}

impl TryFrom<raw::Oversampling> for Oversampling {
    type Error = ReservedValueError;

    fn try_from(value: raw::Oversampling) -> Result<Self, Self::Error> {
        match value {
            raw::Oversampling::X1 => Ok(Self::X1),
            raw::Oversampling::X2 => Ok(Self::X2),
            raw::Oversampling::X4 => Ok(Self::X4),
            raw::Oversampling::X8 => Ok(Self::X8),
            raw::Oversampling::X16 => Ok(Self::X16),
            raw::Oversampling::X32 => Ok(Self::X32),
            raw::Oversampling::Reserved(value) => {
                Err(ReservedValueError::new("oversampling", value))
            }
        }
    }
}

impl From<Oversampling> for raw::Oversampling {
    fn from(value: Oversampling) -> Self {
        match value {
            Oversampling::X1 => Self::X1,
            Oversampling::X2 => Self::X2,
            Oversampling::X4 => Self::X4,
            Oversampling::X8 => Self::X8,
            Oversampling::X16 => Self::X16,
            Oversampling::X32 => Self::X32,
        }
    }
}

/// Pressure and temperature oversampling represented by [`field_sets::Osr`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OversamplingConfig {
    /// Pressure oversampling.
    pub pressure: Oversampling,

    /// Temperature oversampling.
    pub temperature: Oversampling,
}

impl TryFrom<field_sets::Osr> for OversamplingConfig {
    type Error = ReservedValueError;

    fn try_from(value: field_sets::Osr) -> Result<Self, Self::Error> {
        let pressure = value
            .pressure()
            .try_into()
            .map_err(|error: ReservedValueError| {
                ReservedValueError::new("osr.pressure", error.value)
            })?;

        let temperature = value
            .temperature()
            .try_into()
            .map_err(|error: ReservedValueError| {
                ReservedValueError::new("osr.temperature", error.value)
            })?;

        Ok(Self {
            pressure,
            temperature,
        })
    }
}

impl From<OversamplingConfig> for field_sets::Osr {
    fn from(value: OversamplingConfig) -> Self {
        let mut register = Self::new_zero();
        register.set_pressure(value.pressure.into());
        register.set_temperature(value.temperature.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [Oversampling; 6] = [
        Oversampling::X1,
        Oversampling::X2,
        Oversampling::X4,
        Oversampling::X8,
        Oversampling::X16,
        Oversampling::X32,
    ];

    #[test]
    fn oversampling_roundtrips() {
        for value in ALL {
            assert_eq!(
                Oversampling::try_from(raw::Oversampling::from(value)),
                Ok(value)
            );
        }
    }

    #[test]
    fn roundtrips() {
        for pressure in ALL {
            for temperature in ALL {
                let oversampling = OversamplingConfig {
                    pressure,
                    temperature,
                };

                assert_eq!(
                    OversamplingConfig::try_from(field_sets::Osr::from(oversampling)),
                    Ok(oversampling)
                );
            }
        }
    }

    #[test]
    fn error_reports_field() {
        let mut oversampling = field_sets::Osr::new_zero();
        oversampling.set_pressure(raw::Oversampling::Reserved(6));
        assert_eq!(
            OversamplingConfig::try_from(oversampling),
            Err(ReservedValueError::new("osr.pressure", 6))
        );

        let mut oversampling = field_sets::Osr::new_zero();
        oversampling.set_temperature(raw::Oversampling::Reserved(7));
        assert_eq!(
            OversamplingConfig::try_from(oversampling),
            Err(ReservedValueError::new("osr.temperature", 7))
        );
    }
}
