//! Contains extra functionality on top of the generated [`device_driver`] code.

mod defaults {
    //! Default values for register fields based on the default value of the registers they belong to.

    use crate::raw::{
        FifoDataSelect, FifoMode, I2CWatchdogPeriod, IirFilterCoefficient, IntLevel, IntOpenDrain,
        MeasurementMode, OdrSel, Oversampling,
    };

    impl FifoMode {
        pub const DEFAULT: Self = Self::Disabled;
    }

    impl Default for FifoMode {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl FifoDataSelect {
        pub const DEFAULT: Self = Self::Unfiltered;
    }

    impl Default for FifoDataSelect {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl IntOpenDrain {
        /// The default interrupt pin configuration.
        pub const DEFAULT: Self = Self::PushPull;
    }

    impl Default for IntOpenDrain {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl IntLevel {
        /// The default interrupt pin configuration.
        pub const DEFAULT: Self = Self::ActiveLow;
    }

    impl Default for IntLevel {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl I2CWatchdogPeriod {
        pub const DEFAULT: Self = Self::Short;
    }

    impl Default for I2CWatchdogPeriod {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl MeasurementMode {
        /// The default measurment mode.
        pub const DEFAULT: Self = Self::Sleep;
    }

    impl Default for MeasurementMode {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl Oversampling {
        /// The default oversampling rate for pressure measurements.
        pub const DEFAULT_PRESSURE: Self = Self::X4;

        /// The default oversampling rate for temperature measurements.
        pub const DEFAULT_TEMPERATURE: Self = Self::X1;
    }

    impl OdrSel {
        /// The default output data rate.
        pub const DEFAULT: Self = Self::Odr200;
    }

    impl Default for OdrSel {
        fn default() -> Self {
            Self::DEFAULT
        }
    }

    impl IirFilterCoefficient {
        /// The default IIR filter coefficient.
        pub const DEFAULT: Self = Self::Coef0;
    }

    impl Default for IirFilterCoefficient {
        fn default() -> Self {
            Self::DEFAULT
        }
    }
}

#[cfg(feature = "embassy-time")]
mod odr_sel_into_duration {
    use crate::raw::OdrSel;
    use embassy_time::Duration;

    impl OdrSel {
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
            use OdrSel::*;
            match self {
                Odr200 | Reserved(_) => Self::PERIOD_200HZ,
                Odr100 => Self::PERIOD_100HZ,
                Odr50 => Self::PERIOD_50HZ,
                Odr25 => Self::PERIOD_25HZ,
                Odr12P5 => Self::PERIOD_12P5HZ,
                Odr6P25 => Self::PERIOD_6P25HZ,
                Odr3P1 => Self::PERIOD_3P1HZ,
                Odr1P5 => Self::PERIOD_1P5HZ,
                Odr0P78 => Self::PERIOD_0P78HZ,
                Odr0P39 => Self::PERIOD_0P39HZ,
                Odr0P2 => Self::PERIOD_0P2HZ,
                Odr0P1 => Self::PERIOD_0P1HZ,
                Odr0P05 => Self::PERIOD_0P05HZ,
                Odr0P02 => Self::PERIOD_0P02HZ,
                Odr0P01 => Self::PERIOD_0P01HZ,
                Odr0P006 => Self::PERIOD_0P006HZ,
                Odr0P003 => Self::PERIOD_0P003HZ,
                Odr0P0015 => Self::PERIOD_0P0015HZ,
            }
        }
    }

    impl From<OdrSel> for Duration {
        /// Convert this type to an [`Duration`] that represents the sampling period.
        ///
        /// Use [`OdrSel::into_duration`] if you want a `const fn` version of this.
        fn from(odr: OdrSel) -> Duration {
            odr.into_duration()
        }
    }
}
