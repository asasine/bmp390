use super::ReservedValueError;
use crate::raw::{self, field_sets};

/// Output data rate for pressure and temperature measurements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutputDataRate {
    /// 200 Hz, 5 ms sampling period, prescaler 1.
    Hz200,

    /// 100 Hz, 10 ms sampling period, prescaler 2.
    Hz100,

    /// 50 Hz, 20 ms sampling period, prescaler 4.
    Hz50,

    /// 25 Hz, 40 ms sampling period, prescaler 8.
    Hz25,

    /// 25/2 Hz, 80 ms sampling period, prescaler 16.
    Hz12P5,

    /// 25/4 Hz, 160 ms sampling period, prescaler 32.
    Hz6P25,

    /// 25/8 Hz, 320 ms sampling period, prescaler 64.
    Hz3P1,

    /// 25/16 Hz, 640 ms sampling period, prescaler 128.
    Hz1P5,

    /// 25/32 Hz, 1.280 s sampling period, prescaler 256.
    Hz0P78,

    /// 25/64 Hz, 2.560 s sampling period, prescaler 512.
    Hz0P39,

    /// 25/128 Hz, 5.120 s sampling period, prescaler 1024.
    Hz0P2,

    /// 25/256 Hz, 10.24 s sampling period, prescaler 2048.
    Hz0P1,

    /// 25/512 Hz, 20.48 s sampling period, prescaler 4096.
    Hz0P05,

    /// 25/1024 Hz, 40.96 s sampling period, prescaler 8192.
    Hz0P02,

    /// 25/2048 Hz, 81.92 s sampling period, prescaler 16384.
    Hz0P01,

    /// 25/4096 Hz, 163.84 s sampling period, prescaler 32768.
    Hz0P006,

    /// 25/8192 Hz, 327.68 s sampling period, prescaler 65536.
    Hz0P003,

    /// 25/16384 Hz, 655.36 s sampling period, prescaler 131072.
    Hz0P0015,
}

impl TryFrom<raw::OdrSel> for OutputDataRate {
    type Error = ReservedValueError;

    fn try_from(value: raw::OdrSel) -> Result<Self, Self::Error> {
        match value {
            raw::OdrSel::Odr200 => Ok(Self::Hz200),
            raw::OdrSel::Odr100 => Ok(Self::Hz100),
            raw::OdrSel::Odr50 => Ok(Self::Hz50),
            raw::OdrSel::Odr25 => Ok(Self::Hz25),
            raw::OdrSel::Odr12P5 => Ok(Self::Hz12P5),
            raw::OdrSel::Odr6P25 => Ok(Self::Hz6P25),
            raw::OdrSel::Odr3P1 => Ok(Self::Hz3P1),
            raw::OdrSel::Odr1P5 => Ok(Self::Hz1P5),
            raw::OdrSel::Odr0P78 => Ok(Self::Hz0P78),
            raw::OdrSel::Odr0P39 => Ok(Self::Hz0P39),
            raw::OdrSel::Odr0P2 => Ok(Self::Hz0P2),
            raw::OdrSel::Odr0P1 => Ok(Self::Hz0P1),
            raw::OdrSel::Odr0P05 => Ok(Self::Hz0P05),
            raw::OdrSel::Odr0P02 => Ok(Self::Hz0P02),
            raw::OdrSel::Odr0P01 => Ok(Self::Hz0P01),
            raw::OdrSel::Odr0P006 => Ok(Self::Hz0P006),
            raw::OdrSel::Odr0P003 => Ok(Self::Hz0P003),
            raw::OdrSel::Odr0P0015 => Ok(Self::Hz0P0015),
            raw::OdrSel::Reserved(value) => Err(ReservedValueError::new("odr.odr_sel", value)),
        }
    }
}

impl From<OutputDataRate> for raw::OdrSel {
    fn from(value: OutputDataRate) -> Self {
        match value {
            OutputDataRate::Hz200 => Self::Odr200,
            OutputDataRate::Hz100 => Self::Odr100,
            OutputDataRate::Hz50 => Self::Odr50,
            OutputDataRate::Hz25 => Self::Odr25,
            OutputDataRate::Hz12P5 => Self::Odr12P5,
            OutputDataRate::Hz6P25 => Self::Odr6P25,
            OutputDataRate::Hz3P1 => Self::Odr3P1,
            OutputDataRate::Hz1P5 => Self::Odr1P5,
            OutputDataRate::Hz0P78 => Self::Odr0P78,
            OutputDataRate::Hz0P39 => Self::Odr0P39,
            OutputDataRate::Hz0P2 => Self::Odr0P2,
            OutputDataRate::Hz0P1 => Self::Odr0P1,
            OutputDataRate::Hz0P05 => Self::Odr0P05,
            OutputDataRate::Hz0P02 => Self::Odr0P02,
            OutputDataRate::Hz0P01 => Self::Odr0P01,
            OutputDataRate::Hz0P006 => Self::Odr0P006,
            OutputDataRate::Hz0P003 => Self::Odr0P003,
            OutputDataRate::Hz0P0015 => Self::Odr0P0015,
        }
    }
}

impl TryFrom<field_sets::Odr> for OutputDataRate {
    type Error = ReservedValueError;

    fn try_from(value: field_sets::Odr) -> Result<Self, Self::Error> {
        Ok(value.odr_sel().try_into()?)
    }
}

impl From<OutputDataRate> for field_sets::Odr {
    fn from(value: OutputDataRate) -> Self {
        let mut register = Self::new_zero();
        register.set_odr_sel(value.into());
        register
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [OutputDataRate; 18] = [
        OutputDataRate::Hz200,
        OutputDataRate::Hz100,
        OutputDataRate::Hz50,
        OutputDataRate::Hz25,
        OutputDataRate::Hz12P5,
        OutputDataRate::Hz6P25,
        OutputDataRate::Hz3P1,
        OutputDataRate::Hz1P5,
        OutputDataRate::Hz0P78,
        OutputDataRate::Hz0P39,
        OutputDataRate::Hz0P2,
        OutputDataRate::Hz0P1,
        OutputDataRate::Hz0P05,
        OutputDataRate::Hz0P02,
        OutputDataRate::Hz0P01,
        OutputDataRate::Hz0P006,
        OutputDataRate::Hz0P003,
        OutputDataRate::Hz0P0015,
    ];

    #[test]
    fn output_data_rate_roundtrips() {
        for value in ALL {
            assert_eq!(
                OutputDataRate::try_from(raw::OdrSel::from(value)),
                Ok(value)
            );
        }
    }

    #[test]
    fn roundtrips() {
        for rate in ALL {
            let output_data_rate = rate;
            assert_eq!(
                OutputDataRate::try_from(field_sets::Odr::from(output_data_rate)),
                Ok(output_data_rate)
            );
        }
    }

    #[test]
    fn error_reports_field() {
        let mut output_data_rate = field_sets::Odr::new_zero();
        output_data_rate.set_odr_sel(raw::OdrSel::Reserved(31));
        assert_eq!(
            OutputDataRate::try_from(output_data_rate),
            Err(ReservedValueError::new("odr.odr_sel", 31))
        );
    }
}
