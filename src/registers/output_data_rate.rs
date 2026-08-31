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

impl OutputDataRate {
    /// The default output data rate: 200 Hz.
    pub const DEFAULT: Self = Self::Hz200;
}

impl Default for OutputDataRate {
    /// The default output data rate: 200 Hz.
    fn default() -> Self {
        Self::DEFAULT
    }
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
        value.odr_sel().try_into()
    }
}

impl From<OutputDataRate> for field_sets::Odr {
    fn from(value: OutputDataRate) -> Self {
        let mut register = Self::new_zero();
        register.set_odr_sel(value.into());
        register
    }
}

#[cfg(feature = "embassy-time")]
mod into_duration {
    use super::OutputDataRate;
    use embassy_time::Duration;

    impl OutputDataRate {
        /// 5 ms: the sampling period for a 200 Hz output data rate.
        pub const PERIOD_200HZ: Duration = Duration::from_millis(5);

        /// 10 ms: the sampling period for a 100 Hz output data rate.
        pub const PERIOD_100HZ: Duration = Duration::from_millis(10);

        /// 40 ms: the sampling period for a 25 Hz output data rate.
        pub const PERIOD_25HZ: Duration = Duration::from_millis(40);

        /// 160 ms: the sampling period for a 25/4 Hz output data rate.
        pub const PERIOD_6P25HZ: Duration = Duration::from_millis(160);

        /// 640 ms: the sampling period for a 25/16 Hz output data rate.
        pub const PERIOD_1P5HZ: Duration = Duration::from_millis(640);

        /// 2.560 s: the sampling period for a 25/64 Hz output data rate.
        pub const PERIOD_0P39HZ: Duration = Duration::from_millis(2_560);

        /// 10.24 s: the sampling period for a 25/256 Hz output data rate.
        pub const PERIOD_0P1HZ: Duration = Duration::from_millis(10_240);

        /// 40.96 s: the sampling period for a 25/1024 Hz output data rate.
        pub const PERIOD_0P01HZ: Duration = Duration::from_millis(81_920);

        /// 163.84 s: the sampling period for a 25/4096 Hz output data rate.
        pub const PERIOD_0P003HZ: Duration = Duration::from_millis(327_680);

        /// 20 ms: the sampling period for a 50 Hz output data rate.
        pub const PERIOD_50HZ: Duration = Duration::from_millis(20);

        /// 80 ms: the sampling period for a 25/2 Hz output data rate.
        pub const PERIOD_12P5HZ: Duration = Duration::from_millis(80);

        /// 320 ms: the sampling period for a 25/8 Hz output data rate.
        pub const PERIOD_3P1HZ: Duration = Duration::from_millis(320);

        /// 1.280 s: the sampling period for a 25/32 Hz output data rate.
        pub const PERIOD_0P78HZ: Duration = Duration::from_millis(1_280);

        /// 5.120 s: the sampling period for a 25/128 Hz output data rate.
        pub const PERIOD_0P2HZ: Duration = Duration::from_millis(5_120);

        /// 20.48 s: the sampling period for a 25/512 Hz output data rate.
        pub const PERIOD_0P05HZ: Duration = Duration::from_millis(20_480);

        /// 81.92 s: the sampling period for a 25/2048 Hz output data rate.
        pub const PERIOD_0P02HZ: Duration = Duration::from_millis(40_960);

        /// 327.68 s: the sampling period for a 25/8192 Hz output data rate.
        pub const PERIOD_0P006HZ: Duration = Duration::from_millis(163_840);

        /// 655.36 s: the sampling period for a 25/16384 Hz output data rate.
        pub const PERIOD_0P0015HZ: Duration = Duration::from_millis(655_360);

        /// Convert this type to an [`embassy_time::Duration`] that represents the sampling period.
        pub const fn into_duration(self) -> Duration {
            use OutputDataRate::*;
            match self {
                Hz200 => Self::PERIOD_200HZ,
                Hz100 => Self::PERIOD_100HZ,
                Hz50 => Self::PERIOD_50HZ,
                Hz25 => Self::PERIOD_25HZ,
                Hz12P5 => Self::PERIOD_12P5HZ,
                Hz6P25 => Self::PERIOD_6P25HZ,
                Hz3P1 => Self::PERIOD_3P1HZ,
                Hz1P5 => Self::PERIOD_1P5HZ,
                Hz0P78 => Self::PERIOD_0P78HZ,
                Hz0P39 => Self::PERIOD_0P39HZ,
                Hz0P2 => Self::PERIOD_0P2HZ,
                Hz0P1 => Self::PERIOD_0P1HZ,
                Hz0P05 => Self::PERIOD_0P05HZ,
                Hz0P02 => Self::PERIOD_0P02HZ,
                Hz0P01 => Self::PERIOD_0P01HZ,
                Hz0P006 => Self::PERIOD_0P006HZ,
                Hz0P003 => Self::PERIOD_0P003HZ,
                Hz0P0015 => Self::PERIOD_0P0015HZ,
            }
        }
    }

    impl From<OutputDataRate> for Duration {
        /// Convert this type to an [`Duration`] that represents the sampling period.
        ///
        /// Use [`OutputDataRate::into_duration`] if you want a `const fn` version of this.
        fn from(odr: OutputDataRate) -> Duration {
            odr.into_duration()
        }
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
