/// Root block of the Device driver
#[derive(Debug)]
pub struct Device<I> {
    pub(crate) interface: I,
    #[doc(hidden)]
    base_address: u8,
}
impl<I> Device<I> {
    /// Create a new instance of the block based on device interface
    pub const fn new(interface: I) -> Self {
        Self { interface, base_address: 0 }
    }
    /// A reference to the interface used to communicate with the device
    pub(crate) fn interface(&mut self) -> &mut I {
        &mut self.interface
    }
    /// Read all readable register values in this block from the device.
    /// The callback is called for each of them.
    /// Any registers in child blocks are not included.
    ///
    /// The callback has three arguments:
    ///
    /// - The address of the register
    /// - The name of the register (with index for repeated registers)
    /// - The read value from the register
    ///
    /// This is useful for e.g. debug printing all values.
    /// The given [field_sets::FieldSetValue] has a Debug and Format implementation that forwards to the concrete type
    /// the lies within so it can be printed without matching on it.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    pub fn read_all_registers(
        &mut self,
        mut callback: impl FnMut(u8, &'static str, field_sets::FieldSetValue),
    ) -> Result<(), I::Error>
    where
        I: ::device_driver::RegisterInterface<AddressType = u8>,
    {
        let reg = self.chip_id().read()?;
        callback(0 + 0 * 0, "chip_id", reg.into());
        let reg = self.rev_id().read()?;
        callback(1 + 0 * 0, "rev_id", reg.into());
        let reg = self.err_reg().read()?;
        callback(2 + 0 * 0, "err_reg", reg.into());
        let reg = self.status().read()?;
        callback(3 + 0 * 0, "status", reg.into());
        let reg = self.pressure_data().read()?;
        callback(4 + 0 * 0, "pressure_data", reg.into());
        let reg = self.temperature_data().read()?;
        callback(7 + 0 * 0, "temperature_data", reg.into());
        let reg = self.sensor_time().read()?;
        callback(12 + 0 * 0, "sensor_time", reg.into());
        let reg = self.data().read()?;
        callback(4 + 0 * 0, "data", reg.into());
        let reg = self.event().read()?;
        callback(16 + 0 * 0, "event", reg.into());
        let reg = self.int_status().read()?;
        callback(17 + 0 * 0, "int_status", reg.into());
        let reg = self.fifo_length().read()?;
        callback(18 + 0 * 0, "fifo_length", reg.into());
        let reg = self.fifo_data().read()?;
        callback(20 + 0 * 0, "fifo_data", reg.into());
        let reg = self.fifo_watermark().read()?;
        callback(21 + 0 * 0, "fifo_watermark", reg.into());
        let reg = self.fifo_config().read()?;
        callback(23 + 0 * 0, "fifo_config", reg.into());
        let reg = self.int_ctrl().read()?;
        callback(25 + 0 * 0, "int_ctrl", reg.into());
        let reg = self.if_conf().read()?;
        callback(26 + 0 * 0, "if_conf", reg.into());
        let reg = self.pwr_ctrl().read()?;
        callback(27 + 0 * 0, "pwr_ctrl", reg.into());
        let reg = self.osr().read()?;
        callback(28 + 0 * 0, "osr", reg.into());
        let reg = self.odr().read()?;
        callback(29 + 0 * 0, "odr", reg.into());
        let reg = self.config().read()?;
        callback(31 + 0 * 0, "config", reg.into());
        let reg = self.calibration_data().read()?;
        callback(48 + 0 * 0, "calibration_data", reg.into());
        Ok(())
    }
    /// Read all readable register values in this block from the device.
    /// The callback is called for each of them.
    /// Any registers in child blocks are not included.
    ///
    /// The callback has three arguments:
    ///
    /// - The address of the register
    /// - The name of the register (with index for repeated registers)
    /// - The read value from the register
    ///
    /// This is useful for e.g. debug printing all values.
    /// The given [field_sets::FieldSetValue] has a Debug and Format implementation that forwards to the concrete type
    /// the lies within so it can be printed without matching on it.
    #[allow(unused_mut)]
    #[allow(unused_variables)]
    pub async fn read_all_registers_async(
        &mut self,
        mut callback: impl FnMut(u8, &'static str, field_sets::FieldSetValue),
    ) -> Result<(), I::Error>
    where
        I: ::device_driver::AsyncRegisterInterface<AddressType = u8>,
    {
        let reg = self.chip_id().read_async().await?;
        callback(0 + 0 * 0, "chip_id", reg.into());
        let reg = self.rev_id().read_async().await?;
        callback(1 + 0 * 0, "rev_id", reg.into());
        let reg = self.err_reg().read_async().await?;
        callback(2 + 0 * 0, "err_reg", reg.into());
        let reg = self.status().read_async().await?;
        callback(3 + 0 * 0, "status", reg.into());
        let reg = self.pressure_data().read_async().await?;
        callback(4 + 0 * 0, "pressure_data", reg.into());
        let reg = self.temperature_data().read_async().await?;
        callback(7 + 0 * 0, "temperature_data", reg.into());
        let reg = self.sensor_time().read_async().await?;
        callback(12 + 0 * 0, "sensor_time", reg.into());
        let reg = self.data().read_async().await?;
        callback(4 + 0 * 0, "data", reg.into());
        let reg = self.event().read_async().await?;
        callback(16 + 0 * 0, "event", reg.into());
        let reg = self.int_status().read_async().await?;
        callback(17 + 0 * 0, "int_status", reg.into());
        let reg = self.fifo_length().read_async().await?;
        callback(18 + 0 * 0, "fifo_length", reg.into());
        let reg = self.fifo_data().read_async().await?;
        callback(20 + 0 * 0, "fifo_data", reg.into());
        let reg = self.fifo_watermark().read_async().await?;
        callback(21 + 0 * 0, "fifo_watermark", reg.into());
        let reg = self.fifo_config().read_async().await?;
        callback(23 + 0 * 0, "fifo_config", reg.into());
        let reg = self.int_ctrl().read_async().await?;
        callback(25 + 0 * 0, "int_ctrl", reg.into());
        let reg = self.if_conf().read_async().await?;
        callback(26 + 0 * 0, "if_conf", reg.into());
        let reg = self.pwr_ctrl().read_async().await?;
        callback(27 + 0 * 0, "pwr_ctrl", reg.into());
        let reg = self.osr().read_async().await?;
        callback(28 + 0 * 0, "osr", reg.into());
        let reg = self.odr().read_async().await?;
        callback(29 + 0 * 0, "odr", reg.into());
        let reg = self.config().read_async().await?;
        callback(31 + 0 * 0, "config", reg.into());
        let reg = self.calibration_data().read_async().await?;
        callback(48 + 0 * 0, "calibration_data", reg.into());
        Ok(())
    }
    /// The Chip ID register contains the chip identification code.
    pub fn chip_id(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::ChipId,
        ::device_driver::RW,
    > {
        let address = self.base_address + 0;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::ChipId,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::ChipId::new)
    }
    /// The Revision ID register contains the mask revision of the ASIC.
    pub fn rev_id(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::RevId,
        ::device_driver::RW,
    > {
        let address = self.base_address + 1;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::RevId,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::RevId::new)
    }
    /// Sensor error conditions.
    pub fn err_reg(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::ErrReg,
        ::device_driver::RW,
    > {
        let address = self.base_address + 2;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::ErrReg,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::ErrReg::new)
    }
    /// Sensor status flags.
    pub fn status(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Status,
        ::device_driver::RW,
    > {
        let address = self.base_address + 3;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Status,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Status::new)
    }
    /// 24-bit pressure data, split and stored in three consecutive registers.
    pub fn pressure_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::PressureData,
        ::device_driver::RW,
    > {
        let address = self.base_address + 4;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::PressureData,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::PressureData::new)
    }
    /// 24-bit temperature data, split and stored in three consecutive registers.
    pub fn temperature_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::TemperatureData,
        ::device_driver::RW,
    > {
        let address = self.base_address + 7;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::TemperatureData,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::TemperatureData::new)
    }
    /// 24-bit sensor time data, split and stored in three consecutive registers.
    pub fn sensor_time(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::SensorTime,
        ::device_driver::RW,
    > {
        let address = self.base_address + 12;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::SensorTime,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::SensorTime::new)
    }
    /// Pressure and temperature data, split and stored in six consecutive registers. Reading in one burst read ensures that the pressure and temperature data are from the same measurement sampling.
    pub fn data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Data,
        ::device_driver::RW,
    > {
        let address = self.base_address + 4;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Data,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Data::new)
    }
    /// Sensor status flags.
    pub fn event(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Event,
        ::device_driver::RW,
    > {
        let address = self.base_address + 16;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Event,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Event::new)
    }
    /// Interrupt status. Cleared after reading.
    pub fn int_status(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::IntStatus,
        ::device_driver::RW,
    > {
        let address = self.base_address + 17;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::IntStatus,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::IntStatus::new)
    }
    /// Indicates the current fill level of the FIFO buffer.
    pub fn fifo_length(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::FifoLength,
        ::device_driver::RW,
    > {
        let address = self.base_address + 18;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::FifoLength,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::FifoLength::new)
    }
    /// FIFO data output.
    pub fn fifo_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::FifoData,
        ::device_driver::RW,
    > {
        let address = self.base_address + 20;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::FifoData,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::FifoData::new)
    }
    /// The FIFO watermark level.
    pub fn fifo_watermark(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::FifoWatermark,
        ::device_driver::RW,
    > {
        let address = self.base_address + 21;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::FifoWatermark,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::FifoWatermark::new)
    }
    /// FIFO frame content configuration.
    pub fn fifo_config(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::FifoConfig,
        ::device_driver::RW,
    > {
        let address = self.base_address + 23;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::FifoConfig,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::FifoConfig::new)
    }
    /// Interrupt configuration, controlling IntStatus register and the INT pin.
    pub fn int_ctrl(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::IntCtrl,
        ::device_driver::RW,
    > {
        let address = self.base_address + 25;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::IntCtrl,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::IntCtrl::new)
    }
    /// Serial interface settings.
    pub fn if_conf(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::IfConf,
        ::device_driver::RW,
    > {
        let address = self.base_address + 26;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::IfConf,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::IfConf::new)
    }
    /// Enables or disables pressure and temperature measurements and the measurement mode.
    pub fn pwr_ctrl(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::PwrCtrl,
        ::device_driver::RW,
    > {
        let address = self.base_address + 27;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::PwrCtrl,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::PwrCtrl::new)
    }
    /// Controls the oversampling settings for pressure and temperature measurements.
    pub fn osr(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Osr,
        ::device_driver::RW,
    > {
        let address = self.base_address + 28;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Osr,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Osr::new)
    }
    /// Controls the output data rate by means of setting the subdivision/subsampling.
    pub fn odr(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Odr,
        ::device_driver::RW,
    > {
        let address = self.base_address + 29;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Odr,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Odr::new)
    }
    /// IIR filter coefficients.
    pub fn config(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::Config,
        ::device_driver::RW,
    > {
        let address = self.base_address + 31;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::Config,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::Config::new)
    }
    /// Calibration data for pressure and temperature compensation.
    pub fn calibration_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        I,
        u8,
        field_sets::CalibrationData,
        ::device_driver::RW,
    > {
        let address = self.base_address + 48;
        ::device_driver::RegisterOperation::<
            '_,
            I,
            u8,
            field_sets::CalibrationData,
            ::device_driver::RW,
        >::new(self.interface(), address as u8, field_sets::CalibrationData::new)
    }
    pub fn cmd(
        &mut self,
    ) -> ::device_driver::CommandOperation<'_, I, u8, field_sets::CmdFieldsIn, ()> {
        let address = self.base_address + 126;
        ::device_driver::CommandOperation::<
            '_,
            I,
            u8,
            field_sets::CmdFieldsIn,
            (),
        >::new(self.interface(), address as u8)
    }
}
/// Module containing the generated fieldsets of the registers and commands
pub mod field_sets {
    #[allow(unused_imports)]
    use super::*;
    /// The Chip ID register contains the chip identification code.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct ChipId {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for ChipId {
        const SIZE_BITS: u32 = 8;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl ChipId {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [96] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `value` field of the register.
        ///
        /// The chip identification code.
        pub fn value(&self) -> u8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 8)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// The chip identification code.
        pub fn set_value(&mut self, value: u8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 8, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for ChipId {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<ChipId> for [u8; 1] {
        fn from(val: ChipId) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for ChipId {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("ChipId");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for ChipId {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "ChipId {{ ");
            defmt::write!(f, "value: {=u8}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for ChipId {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for ChipId {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for ChipId {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for ChipId {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for ChipId {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for ChipId {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for ChipId {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// The Revision ID register contains the mask revision of the ASIC.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct RevId {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for RevId {
        const SIZE_BITS: u32 = 8;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl RevId {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [1] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `value` field of the register.
        ///
        /// The mask revision of the ASIC.
        pub fn value(&self) -> u8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 8)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// The mask revision of the ASIC.
        pub fn set_value(&mut self, value: u8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 8, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for RevId {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<RevId> for [u8; 1] {
        fn from(val: RevId) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for RevId {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("RevId");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for RevId {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "RevId {{ ");
            defmt::write!(f, "value: {=u8}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for RevId {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for RevId {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for RevId {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for RevId {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for RevId {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for RevId {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for RevId {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Sensor error conditions.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct ErrReg {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for ErrReg {
        const SIZE_BITS: u32 = 3;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl ErrReg {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `fatal_err` field of the register.
        ///
        /// Fatal error.
        pub fn fatal_err(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            raw > 0
        }
        ///Read the `cmd_err` field of the register.
        ///
        /// Command execution failed. Cleared on read.
        pub fn cmd_err(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Read the `conf_err` field of the register.
        ///
        /// Sensor configuration error detected (only working in normal mode). Cleared on read.
        pub fn conf_err(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 2, 3)
            };
            raw > 0
        }
        ///Write the `fatal_err` field of the register.
        ///
        /// Fatal error.
        pub fn set_fatal_err(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `cmd_err` field of the register.
        ///
        /// Command execution failed. Cleared on read.
        pub fn set_cmd_err(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `conf_err` field of the register.
        ///
        /// Sensor configuration error detected (only working in normal mode). Cleared on read.
        pub fn set_conf_err(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 2, 3, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for ErrReg {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<ErrReg> for [u8; 1] {
        fn from(val: ErrReg) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for ErrReg {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("ErrReg");
            d.field("fatal_err", &self.fatal_err());
            d.field("cmd_err", &self.cmd_err());
            d.field("conf_err", &self.conf_err());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for ErrReg {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "ErrReg {{ ");
            defmt::write!(f, "fatal_err: {=bool}, ", & self.fatal_err());
            defmt::write!(f, "cmd_err: {=bool}, ", & self.cmd_err());
            defmt::write!(f, "conf_err: {=bool}, ", & self.conf_err());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for ErrReg {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for ErrReg {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for ErrReg {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for ErrReg {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for ErrReg {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for ErrReg {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for ErrReg {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Sensor status flags.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Status {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for Status {
        const SIZE_BITS: u32 = 7;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Status {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `command_ready` field of the register.
        ///
        /// CMD decoder status.
        pub fn command_ready(&self) -> super::CommandStatus {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 4, 5)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `data_ready_pressure` field of the register.
        ///
        /// Data ready for pressure. Reset when one pressure DATA register is read out.
        pub fn data_ready_pressure(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 5, 6)
            };
            raw > 0
        }
        ///Read the `data_ready_temperature` field of the register.
        ///
        /// Data ready for temperature. Reset when the temperature DATA register is read out.
        pub fn data_ready_temperature(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 6, 7)
            };
            raw > 0
        }
        ///Write the `command_ready` field of the register.
        ///
        /// CMD decoder status.
        pub fn set_command_ready(&mut self, value: super::CommandStatus) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 4, 5, &mut self.bits)
            };
        }
        ///Write the `data_ready_pressure` field of the register.
        ///
        /// Data ready for pressure. Reset when one pressure DATA register is read out.
        pub fn set_data_ready_pressure(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 5, 6, &mut self.bits)
            };
        }
        ///Write the `data_ready_temperature` field of the register.
        ///
        /// Data ready for temperature. Reset when the temperature DATA register is read out.
        pub fn set_data_ready_temperature(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 6, 7, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for Status {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<Status> for [u8; 1] {
        fn from(val: Status) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Status {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Status");
            d.field("command_ready", &self.command_ready());
            d.field("data_ready_pressure", &self.data_ready_pressure());
            d.field("data_ready_temperature", &self.data_ready_temperature());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Status {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Status {{ ");
            defmt::write!(f, "command_ready: {}, ", & self.command_ready());
            defmt::write!(
                f, "data_ready_pressure: {=bool}, ", & self.data_ready_pressure()
            );
            defmt::write!(
                f, "data_ready_temperature: {=bool}, ", & self.data_ready_temperature()
            );
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Status {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Status {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Status {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Status {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Status {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Status {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Status {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// 24-bit pressure data, split and stored in three consecutive registers.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct PressureData {
        /// The internal bits
        bits: [u8; 3],
    }
    impl ::device_driver::FieldSet for PressureData {
        const SIZE_BITS: u32 = 24;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl PressureData {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0, 128] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 3] }
        }
        ///Read the `value` field of the register.
        ///
        /// 24-bit pressure data.
        pub fn value(&self) -> u32 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 24)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// 24-bit pressure data.
        pub fn set_value(&mut self, value: u32) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(raw, 0, 24, &mut self.bits)
            };
        }
    }
    impl From<[u8; 3]> for PressureData {
        fn from(bits: [u8; 3]) -> Self {
            Self { bits }
        }
    }
    impl From<PressureData> for [u8; 3] {
        fn from(val: PressureData) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for PressureData {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("PressureData");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for PressureData {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "PressureData {{ ");
            defmt::write!(f, "value: {=u32}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for PressureData {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for PressureData {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for PressureData {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for PressureData {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for PressureData {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for PressureData {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for PressureData {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// 24-bit temperature data, split and stored in three consecutive registers.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct TemperatureData {
        /// The internal bits
        bits: [u8; 3],
    }
    impl ::device_driver::FieldSet for TemperatureData {
        const SIZE_BITS: u32 = 24;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl TemperatureData {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0, 128] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 3] }
        }
        ///Read the `value` field of the register.
        ///
        /// 24-bit temperature data.
        pub fn value(&self) -> u32 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 24)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// 24-bit temperature data.
        pub fn set_value(&mut self, value: u32) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(raw, 0, 24, &mut self.bits)
            };
        }
    }
    impl From<[u8; 3]> for TemperatureData {
        fn from(bits: [u8; 3]) -> Self {
            Self { bits }
        }
    }
    impl From<TemperatureData> for [u8; 3] {
        fn from(val: TemperatureData) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for TemperatureData {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("TemperatureData");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for TemperatureData {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "TemperatureData {{ ");
            defmt::write!(f, "value: {=u32}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for TemperatureData {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for TemperatureData {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for TemperatureData {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for TemperatureData {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for TemperatureData {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for TemperatureData {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for TemperatureData {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// 24-bit sensor time data, split and stored in three consecutive registers.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct SensorTime {
        /// The internal bits
        bits: [u8; 3],
    }
    impl ::device_driver::FieldSet for SensorTime {
        const SIZE_BITS: u32 = 24;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl SensorTime {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 3] }
        }
        ///Read the `value` field of the register.
        ///
        /// 24-bit sensor time data.
        pub fn value(&self) -> u32 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 24)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// 24-bit sensor time data.
        pub fn set_value(&mut self, value: u32) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(raw, 0, 24, &mut self.bits)
            };
        }
    }
    impl From<[u8; 3]> for SensorTime {
        fn from(bits: [u8; 3]) -> Self {
            Self { bits }
        }
    }
    impl From<SensorTime> for [u8; 3] {
        fn from(val: SensorTime) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for SensorTime {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("SensorTime");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for SensorTime {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "SensorTime {{ ");
            defmt::write!(f, "value: {=u32}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for SensorTime {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for SensorTime {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for SensorTime {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for SensorTime {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for SensorTime {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for SensorTime {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for SensorTime {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Pressure and temperature data, split and stored in six consecutive registers. Reading in one burst read ensures that the pressure and temperature data are from the same measurement sampling.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Data {
        /// The internal bits
        bits: [u8; 6],
    }
    impl ::device_driver::FieldSet for Data {
        const SIZE_BITS: u32 = 48;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Data {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self {
                bits: [0, 0, 128, 0, 0, 128],
            }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 6] }
        }
        ///Read the `pressure` field of the register.
        ///
        /// 24-bit pressure data.
        pub fn pressure(&self) -> u32 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 24)
            };
            raw
        }
        ///Read the `temperature` field of the register.
        ///
        /// 24-bit temperature data.
        pub fn temperature(&self) -> u32 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(&self.bits, 24, 48)
            };
            raw
        }
        ///Write the `pressure` field of the register.
        ///
        /// 24-bit pressure data.
        pub fn set_pressure(&mut self, value: u32) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(raw, 0, 24, &mut self.bits)
            };
        }
        ///Write the `temperature` field of the register.
        ///
        /// 24-bit temperature data.
        pub fn set_temperature(&mut self, value: u32) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u32,
                    ::device_driver::ops::LE,
                >(raw, 24, 48, &mut self.bits)
            };
        }
    }
    impl From<[u8; 6]> for Data {
        fn from(bits: [u8; 6]) -> Self {
            Self { bits }
        }
    }
    impl From<Data> for [u8; 6] {
        fn from(val: Data) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Data {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Data");
            d.field("pressure", &self.pressure());
            d.field("temperature", &self.temperature());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Data {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Data {{ ");
            defmt::write!(f, "pressure: {=u32}, ", & self.pressure());
            defmt::write!(f, "temperature: {=u32}, ", & self.temperature());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Data {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Data {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Data {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Data {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Data {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Data {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Data {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Sensor status flags.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Event {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for Event {
        const SIZE_BITS: u32 = 2;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Event {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [1] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `por_detected` field of the register.
        ///
        /// True after device power up or soft reset. Cleared on read.
        pub fn por_detected(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            raw > 0
        }
        ///Read the `itf_act_pt` field of the register.
        ///
        /// True when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read.
        pub fn itf_act_pt(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Write the `por_detected` field of the register.
        ///
        /// True after device power up or soft reset. Cleared on read.
        pub fn set_por_detected(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `itf_act_pt` field of the register.
        ///
        /// True when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read.
        pub fn set_itf_act_pt(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for Event {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<Event> for [u8; 1] {
        fn from(val: Event) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Event {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Event");
            d.field("por_detected", &self.por_detected());
            d.field("itf_act_pt", &self.itf_act_pt());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Event {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Event {{ ");
            defmt::write!(f, "por_detected: {=bool}, ", & self.por_detected());
            defmt::write!(f, "itf_act_pt: {=bool}, ", & self.itf_act_pt());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Event {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Event {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Event {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Event {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Event {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Event {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Event {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Interrupt status. Cleared after reading.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct IntStatus {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for IntStatus {
        const SIZE_BITS: u32 = 4;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl IntStatus {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `fifo_full_watermark` field of the register.
        ///
        /// The FIFO Watermark has been reached.
        pub fn fifo_full_watermark(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            raw > 0
        }
        ///Read the `fifo_full` field of the register.
        ///
        /// The FIFO is full.
        pub fn fifo_full(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Read the `data_ready` field of the register.
        ///
        /// Data ready for pressure and temperature.
        pub fn data_ready(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 3, 4)
            };
            raw > 0
        }
        ///Write the `fifo_full_watermark` field of the register.
        ///
        /// The FIFO Watermark has been reached.
        pub fn set_fifo_full_watermark(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `fifo_full` field of the register.
        ///
        /// The FIFO is full.
        pub fn set_fifo_full(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `data_ready` field of the register.
        ///
        /// Data ready for pressure and temperature.
        pub fn set_data_ready(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 3, 4, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for IntStatus {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<IntStatus> for [u8; 1] {
        fn from(val: IntStatus) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for IntStatus {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("IntStatus");
            d.field("fifo_full_watermark", &self.fifo_full_watermark());
            d.field("fifo_full", &self.fifo_full());
            d.field("data_ready", &self.data_ready());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for IntStatus {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "IntStatus {{ ");
            defmt::write!(
                f, "fifo_full_watermark: {=bool}, ", & self.fifo_full_watermark()
            );
            defmt::write!(f, "fifo_full: {=bool}, ", & self.fifo_full());
            defmt::write!(f, "data_ready: {=bool}, ", & self.data_ready());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for IntStatus {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for IntStatus {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for IntStatus {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for IntStatus {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for IntStatus {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for IntStatus {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for IntStatus {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Indicates the current fill level of the FIFO buffer.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct FifoLength {
        /// The internal bits
        bits: [u8; 2],
    }
    impl ::device_driver::FieldSet for FifoLength {
        const SIZE_BITS: u32 = 9;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl FifoLength {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
        ///Read the `value` field of the register.
        ///
        /// The current fill level of the FIFO buffer.
        pub fn value(&self) -> u16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 9)
            };
            raw
        }
        ///Write the `value` field of the register.
        ///
        /// The current fill level of the FIFO buffer.
        pub fn set_value(&mut self, value: u16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(raw, 0, 9, &mut self.bits)
            };
        }
    }
    impl From<[u8; 2]> for FifoLength {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }
    impl From<FifoLength> for [u8; 2] {
        fn from(val: FifoLength) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for FifoLength {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("FifoLength");
            d.field("value", &self.value());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for FifoLength {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "FifoLength {{ ");
            defmt::write!(f, "value: {=u16}, ", & self.value());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for FifoLength {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for FifoLength {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for FifoLength {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for FifoLength {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for FifoLength {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for FifoLength {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for FifoLength {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// FIFO data output.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct FifoData {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for FifoData {
        const SIZE_BITS: u32 = 8;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl FifoData {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
    }
    impl From<[u8; 1]> for FifoData {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<FifoData> for [u8; 1] {
        fn from(val: FifoData) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for FifoData {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("FifoData");
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for FifoData {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "FifoData {{ ");
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for FifoData {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for FifoData {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for FifoData {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for FifoData {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for FifoData {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for FifoData {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for FifoData {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// The FIFO watermark level.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct FifoWatermark {
        /// The internal bits
        bits: [u8; 2],
    }
    impl ::device_driver::FieldSet for FifoWatermark {
        const SIZE_BITS: u32 = 9;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl FifoWatermark {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [1, 0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
    }
    impl From<[u8; 2]> for FifoWatermark {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }
    impl From<FifoWatermark> for [u8; 2] {
        fn from(val: FifoWatermark) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for FifoWatermark {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("FifoWatermark");
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for FifoWatermark {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "FifoWatermark {{ ");
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for FifoWatermark {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for FifoWatermark {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for FifoWatermark {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for FifoWatermark {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for FifoWatermark {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for FifoWatermark {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for FifoWatermark {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// FIFO frame content configuration.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct FifoConfig {
        /// The internal bits
        bits: [u8; 2],
    }
    impl ::device_driver::FieldSet for FifoConfig {
        const SIZE_BITS: u32 = 16;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl FifoConfig {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [2, 2] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 2] }
        }
        ///Read the `mode` field of the register.
        ///
        /// Enables or disables the FIFO.
        pub fn mode(&self) -> super::FifoMode {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `stop_on_full` field of the register.
        ///
        /// Whether to stop writing samples into FIFO when it is full. True to stop writing samples into FIFO when it is full.
        pub fn stop_on_full(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Read the `time_enable` field of the register.
        ///
        /// Whether to return sensortime frame after the last valid data frame. True to return sensortime frame after the last valid data frame.
        pub fn time_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 2, 3)
            };
            raw > 0
        }
        ///Read the `pressure_enable` field of the register.
        ///
        /// Whether to store pressure data in FIFO. True to store pressure data in FIFO.
        pub fn pressure_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 3, 4)
            };
            raw > 0
        }
        ///Read the `temperature_enable` field of the register.
        ///
        /// Whether to store temperature data in FIFO. True to store temperature data in FIFO.
        pub fn temperature_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 4, 5)
            };
            raw > 0
        }
        ///Read the `subsampling` field of the register.
        ///
        /// FIFO downsampling selection for pressure and temperature data. Factor is `2^sampling`.
        pub fn subsampling(&self) -> u8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 8, 11)
            };
            raw
        }
        ///Read the `data_select` field of the register.
        ///
        /// Selects the data source for pressure and temperature data.
        pub fn data_select(&self) -> super::FifoDataSelect {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 11, 13)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `mode` field of the register.
        ///
        /// Enables or disables the FIFO.
        pub fn set_mode(&mut self, value: super::FifoMode) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `stop_on_full` field of the register.
        ///
        /// Whether to stop writing samples into FIFO when it is full. True to stop writing samples into FIFO when it is full.
        pub fn set_stop_on_full(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `time_enable` field of the register.
        ///
        /// Whether to return sensortime frame after the last valid data frame. True to return sensortime frame after the last valid data frame.
        pub fn set_time_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 2, 3, &mut self.bits)
            };
        }
        ///Write the `pressure_enable` field of the register.
        ///
        /// Whether to store pressure data in FIFO. True to store pressure data in FIFO.
        pub fn set_pressure_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 3, 4, &mut self.bits)
            };
        }
        ///Write the `temperature_enable` field of the register.
        ///
        /// Whether to store temperature data in FIFO. True to store temperature data in FIFO.
        pub fn set_temperature_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 4, 5, &mut self.bits)
            };
        }
        ///Write the `subsampling` field of the register.
        ///
        /// FIFO downsampling selection for pressure and temperature data. Factor is `2^sampling`.
        pub fn set_subsampling(&mut self, value: u8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 8, 11, &mut self.bits)
            };
        }
        ///Write the `data_select` field of the register.
        ///
        /// Selects the data source for pressure and temperature data.
        pub fn set_data_select(&mut self, value: super::FifoDataSelect) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 11, 13, &mut self.bits)
            };
        }
    }
    impl From<[u8; 2]> for FifoConfig {
        fn from(bits: [u8; 2]) -> Self {
            Self { bits }
        }
    }
    impl From<FifoConfig> for [u8; 2] {
        fn from(val: FifoConfig) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for FifoConfig {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("FifoConfig");
            d.field("mode", &self.mode());
            d.field("stop_on_full", &self.stop_on_full());
            d.field("time_enable", &self.time_enable());
            d.field("pressure_enable", &self.pressure_enable());
            d.field("temperature_enable", &self.temperature_enable());
            d.field("subsampling", &self.subsampling());
            d.field("data_select", &self.data_select());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for FifoConfig {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "FifoConfig {{ ");
            defmt::write!(f, "mode: {}, ", & self.mode());
            defmt::write!(f, "stop_on_full: {=bool}, ", & self.stop_on_full());
            defmt::write!(f, "time_enable: {=bool}, ", & self.time_enable());
            defmt::write!(f, "pressure_enable: {=bool}, ", & self.pressure_enable());
            defmt::write!(
                f, "temperature_enable: {=bool}, ", & self.temperature_enable()
            );
            defmt::write!(f, "subsampling: {=u8}, ", & self.subsampling());
            defmt::write!(f, "data_select: {}, ", & self.data_select());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for FifoConfig {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for FifoConfig {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for FifoConfig {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for FifoConfig {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for FifoConfig {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for FifoConfig {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for FifoConfig {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Interrupt configuration, controlling IntStatus register and the INT pin.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct IntCtrl {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for IntCtrl {
        const SIZE_BITS: u32 = 7;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl IntCtrl {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [2] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `int_open_drain` field of the register.
        ///
        /// Configures the INT pin as push-pull or open-drain.
        pub fn int_open_drain(&self) -> super::IntOpenDrain {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `int_level` field of the register.
        ///
        /// Configures the INT pin as active high or active low.
        pub fn int_level(&self) -> super::IntLevel {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `int_latch` field of the register.
        ///
        /// Configures the INT pin as latched or pulsed. True for latched.
        pub fn int_latch(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 2, 3)
            };
            raw > 0
        }
        ///Read the `fifo_watermark_int_enable` field of the register.
        ///
        /// Whether to interrupt when the FIFO watermark is reached. True to interrupt when the FIFO watermark is reached.
        pub fn fifo_watermark_int_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 3, 4)
            };
            raw > 0
        }
        ///Read the `fifo_full_int_enable` field of the register.
        ///
        /// Whether to interrupt when the FIFO is full. True to interrupt when the FIFO is full.
        pub fn fifo_full_int_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 4, 5)
            };
            raw > 0
        }
        ///Read the `int_ds` field of the register.
        ///
        pub fn int_ds(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 5, 6)
            };
            raw > 0
        }
        ///Read the `data_ready_int_enable` field of the register.
        ///
        /// Whether to interrupt when data is ready for pressure and temperature. True to interrupt when data is ready for pressure and temperature.
        pub fn data_ready_int_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 6, 7)
            };
            raw > 0
        }
        ///Write the `int_open_drain` field of the register.
        ///
        /// Configures the INT pin as push-pull or open-drain.
        pub fn set_int_open_drain(&mut self, value: super::IntOpenDrain) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `int_level` field of the register.
        ///
        /// Configures the INT pin as active high or active low.
        pub fn set_int_level(&mut self, value: super::IntLevel) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `int_latch` field of the register.
        ///
        /// Configures the INT pin as latched or pulsed. True for latched.
        pub fn set_int_latch(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 2, 3, &mut self.bits)
            };
        }
        ///Write the `fifo_watermark_int_enable` field of the register.
        ///
        /// Whether to interrupt when the FIFO watermark is reached. True to interrupt when the FIFO watermark is reached.
        pub fn set_fifo_watermark_int_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 3, 4, &mut self.bits)
            };
        }
        ///Write the `fifo_full_int_enable` field of the register.
        ///
        /// Whether to interrupt when the FIFO is full. True to interrupt when the FIFO is full.
        pub fn set_fifo_full_int_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 4, 5, &mut self.bits)
            };
        }
        ///Write the `int_ds` field of the register.
        ///
        pub fn set_int_ds(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 5, 6, &mut self.bits)
            };
        }
        ///Write the `data_ready_int_enable` field of the register.
        ///
        /// Whether to interrupt when data is ready for pressure and temperature. True to interrupt when data is ready for pressure and temperature.
        pub fn set_data_ready_int_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 6, 7, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for IntCtrl {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<IntCtrl> for [u8; 1] {
        fn from(val: IntCtrl) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for IntCtrl {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("IntCtrl");
            d.field("int_open_drain", &self.int_open_drain());
            d.field("int_level", &self.int_level());
            d.field("int_latch", &self.int_latch());
            d.field("fifo_watermark_int_enable", &self.fifo_watermark_int_enable());
            d.field("fifo_full_int_enable", &self.fifo_full_int_enable());
            d.field("int_ds", &self.int_ds());
            d.field("data_ready_int_enable", &self.data_ready_int_enable());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for IntCtrl {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "IntCtrl {{ ");
            defmt::write!(f, "int_open_drain: {}, ", & self.int_open_drain());
            defmt::write!(f, "int_level: {}, ", & self.int_level());
            defmt::write!(f, "int_latch: {=bool}, ", & self.int_latch());
            defmt::write!(
                f, "fifo_watermark_int_enable: {=bool}, ", & self
                .fifo_watermark_int_enable()
            );
            defmt::write!(
                f, "fifo_full_int_enable: {=bool}, ", & self.fifo_full_int_enable()
            );
            defmt::write!(f, "int_ds: {=bool}, ", & self.int_ds());
            defmt::write!(
                f, "data_ready_int_enable: {=bool}, ", & self.data_ready_int_enable()
            );
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for IntCtrl {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for IntCtrl {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for IntCtrl {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for IntCtrl {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for IntCtrl {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for IntCtrl {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for IntCtrl {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Serial interface settings.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct IfConf {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for IfConf {
        const SIZE_BITS: u32 = 3;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl IfConf {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `spi` field of the register.
        ///
        /// Configure SPI Interface Mode for primary interface.
        pub fn spi(&self) -> super::SpiMode {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `i_2_c_wdt_en` field of the register.
        ///
        /// Whether to enable the I2C Watchdog timer, backed by NVM. True to enable.
        pub fn i_2_c_wdt_en(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Read the `i_2_c_wdt_sel` field of the register.
        ///
        /// Selects the timer period for the I2C Watchdog, backed by NVM.
        pub fn i_2_c_wdt_sel(&self) -> super::I2CWatchdogPeriod {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 2, 3)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `spi` field of the register.
        ///
        /// Configure SPI Interface Mode for primary interface.
        pub fn set_spi(&mut self, value: super::SpiMode) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `i_2_c_wdt_en` field of the register.
        ///
        /// Whether to enable the I2C Watchdog timer, backed by NVM. True to enable.
        pub fn set_i_2_c_wdt_en(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `i_2_c_wdt_sel` field of the register.
        ///
        /// Selects the timer period for the I2C Watchdog, backed by NVM.
        pub fn set_i_2_c_wdt_sel(&mut self, value: super::I2CWatchdogPeriod) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 2, 3, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for IfConf {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<IfConf> for [u8; 1] {
        fn from(val: IfConf) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for IfConf {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("IfConf");
            d.field("spi", &self.spi());
            d.field("i_2_c_wdt_en", &self.i_2_c_wdt_en());
            d.field("i_2_c_wdt_sel", &self.i_2_c_wdt_sel());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for IfConf {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "IfConf {{ ");
            defmt::write!(f, "spi: {}, ", & self.spi());
            defmt::write!(f, "i_2_c_wdt_en: {=bool}, ", & self.i_2_c_wdt_en());
            defmt::write!(f, "i_2_c_wdt_sel: {}, ", & self.i_2_c_wdt_sel());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for IfConf {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for IfConf {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for IfConf {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for IfConf {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for IfConf {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for IfConf {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for IfConf {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Enables or disables pressure and temperature measurements and the measurement mode.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct PwrCtrl {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for PwrCtrl {
        const SIZE_BITS: u32 = 6;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl PwrCtrl {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `pressure_enable` field of the register.
        ///
        /// Whether to enable pressure measurements. True to enable.
        pub fn pressure_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 1)
            };
            raw > 0
        }
        ///Read the `temperature_enable` field of the register.
        ///
        /// Whether to enable temperature measurements. True to enable.
        pub fn temperature_enable(&self) -> bool {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 2)
            };
            raw > 0
        }
        ///Read the `mode` field of the register.
        ///
        /// Selects the measurement mode.
        pub fn mode(&self) -> super::MeasurementMode {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 4, 6)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `pressure_enable` field of the register.
        ///
        /// Whether to enable pressure measurements. True to enable.
        pub fn set_pressure_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 1, &mut self.bits)
            };
        }
        ///Write the `temperature_enable` field of the register.
        ///
        /// Whether to enable temperature measurements. True to enable.
        pub fn set_temperature_enable(&mut self, value: bool) {
            let raw = value as _;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 2, &mut self.bits)
            };
        }
        ///Write the `mode` field of the register.
        ///
        /// Selects the measurement mode.
        pub fn set_mode(&mut self, value: super::MeasurementMode) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 4, 6, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for PwrCtrl {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<PwrCtrl> for [u8; 1] {
        fn from(val: PwrCtrl) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for PwrCtrl {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("PwrCtrl");
            d.field("pressure_enable", &self.pressure_enable());
            d.field("temperature_enable", &self.temperature_enable());
            d.field("mode", &self.mode());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for PwrCtrl {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "PwrCtrl {{ ");
            defmt::write!(f, "pressure_enable: {=bool}, ", & self.pressure_enable());
            defmt::write!(
                f, "temperature_enable: {=bool}, ", & self.temperature_enable()
            );
            defmt::write!(f, "mode: {}, ", & self.mode());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for PwrCtrl {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for PwrCtrl {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for PwrCtrl {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for PwrCtrl {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for PwrCtrl {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for PwrCtrl {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for PwrCtrl {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Controls the oversampling settings for pressure and temperature measurements.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Osr {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for Osr {
        const SIZE_BITS: u32 = 6;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Osr {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [2] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `pressure` field of the register.
        ///
        /// Selects the oversampling rate for pressure measurements.
        pub fn pressure(&self) -> super::Oversampling {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 3)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Read the `temperature` field of the register.
        ///
        /// Selects the oversampling rate for temperature measurements.
        pub fn temperature(&self) -> super::Oversampling {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 3, 6)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `pressure` field of the register.
        ///
        /// Selects the oversampling rate for pressure measurements.
        pub fn set_pressure(&mut self, value: super::Oversampling) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 3, &mut self.bits)
            };
        }
        ///Write the `temperature` field of the register.
        ///
        /// Selects the oversampling rate for temperature measurements.
        pub fn set_temperature(&mut self, value: super::Oversampling) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 3, 6, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for Osr {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<Osr> for [u8; 1] {
        fn from(val: Osr) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Osr {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Osr");
            d.field("pressure", &self.pressure());
            d.field("temperature", &self.temperature());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Osr {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Osr {{ ");
            defmt::write!(f, "pressure: {}, ", & self.pressure());
            defmt::write!(f, "temperature: {}, ", & self.temperature());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Osr {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Osr {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Osr {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Osr {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Osr {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Osr {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Osr {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Controls the output data rate by means of setting the subdivision/subsampling.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Odr {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for Odr {
        const SIZE_BITS: u32 = 5;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Odr {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `odr_sel` field of the register.
        ///
        /// Subdivision factor for pressure and temperature measurement is `2^value`.
        pub fn odr_sel(&self) -> super::OdrSel {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 5)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `odr_sel` field of the register.
        ///
        /// Subdivision factor for pressure and temperature measurement is `2^value`.
        pub fn set_odr_sel(&mut self, value: super::OdrSel) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 5, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for Odr {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<Odr> for [u8; 1] {
        fn from(val: Odr) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Odr {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Odr");
            d.field("odr_sel", &self.odr_sel());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Odr {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Odr {{ ");
            defmt::write!(f, "odr_sel: {}, ", & self.odr_sel());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Odr {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Odr {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Odr {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Odr {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Odr {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Odr {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Odr {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// IIR filter coefficients.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct Config {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for Config {
        const SIZE_BITS: u32 = 4;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl Config {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `iir_filter` field of the register.
        ///
        /// Filter coefficient for IIR filter.
        pub fn iir_filter(&self) -> super::IirFilterCoefficient {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 1, 4)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `iir_filter` field of the register.
        ///
        /// Filter coefficient for IIR filter.
        pub fn set_iir_filter(&mut self, value: super::IirFilterCoefficient) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 1, 4, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for Config {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<Config> for [u8; 1] {
        fn from(val: Config) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for Config {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("Config");
            d.field("iir_filter", &self.iir_filter());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for Config {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "Config {{ ");
            defmt::write!(f, "iir_filter: {}, ", & self.iir_filter());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for Config {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for Config {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for Config {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for Config {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for Config {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for Config {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for Config {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Calibration data for pressure and temperature compensation.
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct CalibrationData {
        /// The internal bits
        bits: [u8; 22],
    }
    impl ::device_driver::FieldSet for CalibrationData {
        const SIZE_BITS: u32 = 176;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl CalibrationData {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self {
                bits: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 22] }
        }
        ///Read the `nvm_par_t_1` field of the register.
        ///
        pub fn nvm_par_t_1(&self) -> u16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(&self.bits, 8, 24)
            };
            raw
        }
        ///Read the `nvm_par_t_2` field of the register.
        ///
        pub fn nvm_par_t_2(&self) -> u16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(&self.bits, 24, 40)
            };
            raw
        }
        ///Read the `nvm_par_t_3` field of the register.
        ///
        pub fn nvm_par_t_3(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 40, 48)
            };
            raw
        }
        ///Read the `nvm_par_p_1` field of the register.
        ///
        pub fn nvm_par_p_1(&self) -> i16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(&self.bits, 48, 64)
            };
            raw
        }
        ///Read the `nvm_par_p_2` field of the register.
        ///
        pub fn nvm_par_p_2(&self) -> i16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(&self.bits, 64, 80)
            };
            raw
        }
        ///Read the `nvm_par_p_3` field of the register.
        ///
        pub fn nvm_par_p_3(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 80, 88)
            };
            raw
        }
        ///Read the `nvm_par_p_4` field of the register.
        ///
        pub fn nvm_par_p_4(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 88, 96)
            };
            raw
        }
        ///Read the `nvm_par_p_5` field of the register.
        ///
        pub fn nvm_par_p_5(&self) -> u16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(&self.bits, 96, 112)
            };
            raw
        }
        ///Read the `nvm_par_p_6` field of the register.
        ///
        pub fn nvm_par_p_6(&self) -> u16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(&self.bits, 112, 128)
            };
            raw
        }
        ///Read the `nvm_par_p_7` field of the register.
        ///
        pub fn nvm_par_p_7(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 128, 136)
            };
            raw
        }
        ///Read the `nvm_par_p_8` field of the register.
        ///
        pub fn nvm_par_p_8(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 136, 144)
            };
            raw
        }
        ///Read the `nvm_par_p_9` field of the register.
        ///
        pub fn nvm_par_p_9(&self) -> i16 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(&self.bits, 144, 160)
            };
            raw
        }
        ///Read the `nvm_par_p_10` field of the register.
        ///
        pub fn nvm_par_p_10(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 160, 168)
            };
            raw
        }
        ///Read the `nvm_par_p_11` field of the register.
        ///
        pub fn nvm_par_p_11(&self) -> i8 {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(&self.bits, 168, 176)
            };
            raw
        }
        ///Write the `nvm_par_t_1` field of the register.
        ///
        pub fn set_nvm_par_t_1(&mut self, value: u16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(raw, 8, 24, &mut self.bits)
            };
        }
        ///Write the `nvm_par_t_2` field of the register.
        ///
        pub fn set_nvm_par_t_2(&mut self, value: u16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(raw, 24, 40, &mut self.bits)
            };
        }
        ///Write the `nvm_par_t_3` field of the register.
        ///
        pub fn set_nvm_par_t_3(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 40, 48, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_1` field of the register.
        ///
        pub fn set_nvm_par_p_1(&mut self, value: i16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(raw, 48, 64, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_2` field of the register.
        ///
        pub fn set_nvm_par_p_2(&mut self, value: i16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(raw, 64, 80, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_3` field of the register.
        ///
        pub fn set_nvm_par_p_3(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 80, 88, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_4` field of the register.
        ///
        pub fn set_nvm_par_p_4(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 88, 96, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_5` field of the register.
        ///
        pub fn set_nvm_par_p_5(&mut self, value: u16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(raw, 96, 112, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_6` field of the register.
        ///
        pub fn set_nvm_par_p_6(&mut self, value: u16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u16,
                    ::device_driver::ops::LE,
                >(raw, 112, 128, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_7` field of the register.
        ///
        pub fn set_nvm_par_p_7(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 128, 136, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_8` field of the register.
        ///
        pub fn set_nvm_par_p_8(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 136, 144, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_9` field of the register.
        ///
        pub fn set_nvm_par_p_9(&mut self, value: i16) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i16,
                    ::device_driver::ops::LE,
                >(raw, 144, 160, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_10` field of the register.
        ///
        pub fn set_nvm_par_p_10(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 160, 168, &mut self.bits)
            };
        }
        ///Write the `nvm_par_p_11` field of the register.
        ///
        pub fn set_nvm_par_p_11(&mut self, value: i8) {
            let raw = value;
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    i8,
                    ::device_driver::ops::LE,
                >(raw, 168, 176, &mut self.bits)
            };
        }
    }
    impl From<[u8; 22]> for CalibrationData {
        fn from(bits: [u8; 22]) -> Self {
            Self { bits }
        }
    }
    impl From<CalibrationData> for [u8; 22] {
        fn from(val: CalibrationData) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for CalibrationData {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("CalibrationData");
            d.field("nvm_par_t_1", &self.nvm_par_t_1());
            d.field("nvm_par_t_2", &self.nvm_par_t_2());
            d.field("nvm_par_t_3", &self.nvm_par_t_3());
            d.field("nvm_par_p_1", &self.nvm_par_p_1());
            d.field("nvm_par_p_2", &self.nvm_par_p_2());
            d.field("nvm_par_p_3", &self.nvm_par_p_3());
            d.field("nvm_par_p_4", &self.nvm_par_p_4());
            d.field("nvm_par_p_5", &self.nvm_par_p_5());
            d.field("nvm_par_p_6", &self.nvm_par_p_6());
            d.field("nvm_par_p_7", &self.nvm_par_p_7());
            d.field("nvm_par_p_8", &self.nvm_par_p_8());
            d.field("nvm_par_p_9", &self.nvm_par_p_9());
            d.field("nvm_par_p_10", &self.nvm_par_p_10());
            d.field("nvm_par_p_11", &self.nvm_par_p_11());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for CalibrationData {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "CalibrationData {{ ");
            defmt::write!(f, "nvm_par_t_1: {=u16}, ", & self.nvm_par_t_1());
            defmt::write!(f, "nvm_par_t_2: {=u16}, ", & self.nvm_par_t_2());
            defmt::write!(f, "nvm_par_t_3: {=i8}, ", & self.nvm_par_t_3());
            defmt::write!(f, "nvm_par_p_1: {=i16}, ", & self.nvm_par_p_1());
            defmt::write!(f, "nvm_par_p_2: {=i16}, ", & self.nvm_par_p_2());
            defmt::write!(f, "nvm_par_p_3: {=i8}, ", & self.nvm_par_p_3());
            defmt::write!(f, "nvm_par_p_4: {=i8}, ", & self.nvm_par_p_4());
            defmt::write!(f, "nvm_par_p_5: {=u16}, ", & self.nvm_par_p_5());
            defmt::write!(f, "nvm_par_p_6: {=u16}, ", & self.nvm_par_p_6());
            defmt::write!(f, "nvm_par_p_7: {=i8}, ", & self.nvm_par_p_7());
            defmt::write!(f, "nvm_par_p_8: {=i8}, ", & self.nvm_par_p_8());
            defmt::write!(f, "nvm_par_p_9: {=i16}, ", & self.nvm_par_p_9());
            defmt::write!(f, "nvm_par_p_10: {=i8}, ", & self.nvm_par_p_10());
            defmt::write!(f, "nvm_par_p_11: {=i8}, ", & self.nvm_par_p_11());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for CalibrationData {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for CalibrationData {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for CalibrationData {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for CalibrationData {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for CalibrationData {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for CalibrationData {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for CalibrationData {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    #[derive(Copy, Clone, Eq, PartialEq)]
    pub struct CmdFieldsIn {
        /// The internal bits
        bits: [u8; 1],
    }
    impl ::device_driver::FieldSet for CmdFieldsIn {
        const SIZE_BITS: u32 = 8;
        fn new_with_zero() -> Self {
            Self::new_zero()
        }
        fn get_inner_buffer(&self) -> &[u8] {
            &self.bits
        }
        fn get_inner_buffer_mut(&mut self) -> &mut [u8] {
            &mut self.bits
        }
    }
    impl CmdFieldsIn {
        /// Create a new instance, loaded with the reset value (if any)
        pub const fn new() -> Self {
            Self { bits: [0] }
        }
        /// Create a new instance, loaded with all zeroes
        pub const fn new_zero() -> Self {
            Self { bits: [0; 1] }
        }
        ///Read the `cmd` field of the register.
        ///
        /// Command to execute.
        pub fn cmd(&self) -> super::Command {
            let raw = unsafe {
                ::device_driver::ops::load_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(&self.bits, 0, 8)
            };
            unsafe { raw.try_into().unwrap_unchecked() }
        }
        ///Write the `cmd` field of the register.
        ///
        /// Command to execute.
        pub fn set_cmd(&mut self, value: super::Command) {
            let raw = value.into();
            unsafe {
                ::device_driver::ops::store_lsb0::<
                    u8,
                    ::device_driver::ops::LE,
                >(raw, 0, 8, &mut self.bits)
            };
        }
    }
    impl From<[u8; 1]> for CmdFieldsIn {
        fn from(bits: [u8; 1]) -> Self {
            Self { bits }
        }
    }
    impl From<CmdFieldsIn> for [u8; 1] {
        fn from(val: CmdFieldsIn) -> Self {
            val.bits
        }
    }
    impl core::fmt::Debug for CmdFieldsIn {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
            let mut d = f.debug_struct("CmdFieldsIn");
            d.field("cmd", &self.cmd());
            d.finish()
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for CmdFieldsIn {
        fn format(&self, f: defmt::Formatter) {
            defmt::write!(f, "CmdFieldsIn {{ ");
            defmt::write!(f, "cmd: {}, ", & self.cmd());
            defmt::write!(f, "}}");
        }
    }
    impl core::ops::BitAnd for CmdFieldsIn {
        type Output = Self;
        fn bitand(mut self, rhs: Self) -> Self::Output {
            self &= rhs;
            self
        }
    }
    impl core::ops::BitAndAssign for CmdFieldsIn {
        fn bitand_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l &= *r;
            }
        }
    }
    impl core::ops::BitOr for CmdFieldsIn {
        type Output = Self;
        fn bitor(mut self, rhs: Self) -> Self::Output {
            self |= rhs;
            self
        }
    }
    impl core::ops::BitOrAssign for CmdFieldsIn {
        fn bitor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l |= *r;
            }
        }
    }
    impl core::ops::BitXor for CmdFieldsIn {
        type Output = Self;
        fn bitxor(mut self, rhs: Self) -> Self::Output {
            self ^= rhs;
            self
        }
    }
    impl core::ops::BitXorAssign for CmdFieldsIn {
        fn bitxor_assign(&mut self, rhs: Self) {
            for (l, r) in self.bits.iter_mut().zip(&rhs.bits) {
                *l ^= *r;
            }
        }
    }
    impl core::ops::Not for CmdFieldsIn {
        type Output = Self;
        fn not(mut self) -> Self::Output {
            for val in self.bits.iter_mut() {
                *val = !*val;
            }
            self
        }
    }
    /// Enum containing all possible field set types
    pub enum FieldSetValue {
        /// The Chip ID register contains the chip identification code.
        ChipId(ChipId),
        /// The Revision ID register contains the mask revision of the ASIC.
        RevId(RevId),
        /// Sensor error conditions.
        ErrReg(ErrReg),
        /// Sensor status flags.
        Status(Status),
        /// 24-bit pressure data, split and stored in three consecutive registers.
        PressureData(PressureData),
        /// 24-bit temperature data, split and stored in three consecutive registers.
        TemperatureData(TemperatureData),
        /// 24-bit sensor time data, split and stored in three consecutive registers.
        SensorTime(SensorTime),
        /// Pressure and temperature data, split and stored in six consecutive registers. Reading in one burst read ensures that the pressure and temperature data are from the same measurement sampling.
        Data(Data),
        /// Sensor status flags.
        Event(Event),
        /// Interrupt status. Cleared after reading.
        IntStatus(IntStatus),
        /// Indicates the current fill level of the FIFO buffer.
        FifoLength(FifoLength),
        /// FIFO data output.
        FifoData(FifoData),
        /// The FIFO watermark level.
        FifoWatermark(FifoWatermark),
        /// FIFO frame content configuration.
        FifoConfig(FifoConfig),
        /// Interrupt configuration, controlling IntStatus register and the INT pin.
        IntCtrl(IntCtrl),
        /// Serial interface settings.
        IfConf(IfConf),
        /// Enables or disables pressure and temperature measurements and the measurement mode.
        PwrCtrl(PwrCtrl),
        /// Controls the oversampling settings for pressure and temperature measurements.
        Osr(Osr),
        /// Controls the output data rate by means of setting the subdivision/subsampling.
        Odr(Odr),
        /// IIR filter coefficients.
        Config(Config),
        /// Calibration data for pressure and temperature compensation.
        CalibrationData(CalibrationData),
        CmdFieldsIn(CmdFieldsIn),
    }
    impl core::fmt::Debug for FieldSetValue {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            match self {
                Self::ChipId(val) => core::fmt::Debug::fmt(val, f),
                Self::RevId(val) => core::fmt::Debug::fmt(val, f),
                Self::ErrReg(val) => core::fmt::Debug::fmt(val, f),
                Self::Status(val) => core::fmt::Debug::fmt(val, f),
                Self::PressureData(val) => core::fmt::Debug::fmt(val, f),
                Self::TemperatureData(val) => core::fmt::Debug::fmt(val, f),
                Self::SensorTime(val) => core::fmt::Debug::fmt(val, f),
                Self::Data(val) => core::fmt::Debug::fmt(val, f),
                Self::Event(val) => core::fmt::Debug::fmt(val, f),
                Self::IntStatus(val) => core::fmt::Debug::fmt(val, f),
                Self::FifoLength(val) => core::fmt::Debug::fmt(val, f),
                Self::FifoData(val) => core::fmt::Debug::fmt(val, f),
                Self::FifoWatermark(val) => core::fmt::Debug::fmt(val, f),
                Self::FifoConfig(val) => core::fmt::Debug::fmt(val, f),
                Self::IntCtrl(val) => core::fmt::Debug::fmt(val, f),
                Self::IfConf(val) => core::fmt::Debug::fmt(val, f),
                Self::PwrCtrl(val) => core::fmt::Debug::fmt(val, f),
                Self::Osr(val) => core::fmt::Debug::fmt(val, f),
                Self::Odr(val) => core::fmt::Debug::fmt(val, f),
                Self::Config(val) => core::fmt::Debug::fmt(val, f),
                Self::CalibrationData(val) => core::fmt::Debug::fmt(val, f),
                Self::CmdFieldsIn(val) => core::fmt::Debug::fmt(val, f),
                #[allow(unreachable_patterns)]
                _ => unreachable!(),
            }
        }
    }
    #[cfg(feature = "defmt")]
    impl defmt::Format for FieldSetValue {
        fn format(&self, f: defmt::Formatter) {
            match self {
                Self::ChipId(val) => defmt::Format::format(val, f),
                Self::RevId(val) => defmt::Format::format(val, f),
                Self::ErrReg(val) => defmt::Format::format(val, f),
                Self::Status(val) => defmt::Format::format(val, f),
                Self::PressureData(val) => defmt::Format::format(val, f),
                Self::TemperatureData(val) => defmt::Format::format(val, f),
                Self::SensorTime(val) => defmt::Format::format(val, f),
                Self::Data(val) => defmt::Format::format(val, f),
                Self::Event(val) => defmt::Format::format(val, f),
                Self::IntStatus(val) => defmt::Format::format(val, f),
                Self::FifoLength(val) => defmt::Format::format(val, f),
                Self::FifoData(val) => defmt::Format::format(val, f),
                Self::FifoWatermark(val) => defmt::Format::format(val, f),
                Self::FifoConfig(val) => defmt::Format::format(val, f),
                Self::IntCtrl(val) => defmt::Format::format(val, f),
                Self::IfConf(val) => defmt::Format::format(val, f),
                Self::PwrCtrl(val) => defmt::Format::format(val, f),
                Self::Osr(val) => defmt::Format::format(val, f),
                Self::Odr(val) => defmt::Format::format(val, f),
                Self::Config(val) => defmt::Format::format(val, f),
                Self::CalibrationData(val) => defmt::Format::format(val, f),
                Self::CmdFieldsIn(val) => defmt::Format::format(val, f),
            }
        }
    }
    impl From<ChipId> for FieldSetValue {
        fn from(val: ChipId) -> Self {
            Self::ChipId(val)
        }
    }
    impl From<RevId> for FieldSetValue {
        fn from(val: RevId) -> Self {
            Self::RevId(val)
        }
    }
    impl From<ErrReg> for FieldSetValue {
        fn from(val: ErrReg) -> Self {
            Self::ErrReg(val)
        }
    }
    impl From<Status> for FieldSetValue {
        fn from(val: Status) -> Self {
            Self::Status(val)
        }
    }
    impl From<PressureData> for FieldSetValue {
        fn from(val: PressureData) -> Self {
            Self::PressureData(val)
        }
    }
    impl From<TemperatureData> for FieldSetValue {
        fn from(val: TemperatureData) -> Self {
            Self::TemperatureData(val)
        }
    }
    impl From<SensorTime> for FieldSetValue {
        fn from(val: SensorTime) -> Self {
            Self::SensorTime(val)
        }
    }
    impl From<Data> for FieldSetValue {
        fn from(val: Data) -> Self {
            Self::Data(val)
        }
    }
    impl From<Event> for FieldSetValue {
        fn from(val: Event) -> Self {
            Self::Event(val)
        }
    }
    impl From<IntStatus> for FieldSetValue {
        fn from(val: IntStatus) -> Self {
            Self::IntStatus(val)
        }
    }
    impl From<FifoLength> for FieldSetValue {
        fn from(val: FifoLength) -> Self {
            Self::FifoLength(val)
        }
    }
    impl From<FifoData> for FieldSetValue {
        fn from(val: FifoData) -> Self {
            Self::FifoData(val)
        }
    }
    impl From<FifoWatermark> for FieldSetValue {
        fn from(val: FifoWatermark) -> Self {
            Self::FifoWatermark(val)
        }
    }
    impl From<FifoConfig> for FieldSetValue {
        fn from(val: FifoConfig) -> Self {
            Self::FifoConfig(val)
        }
    }
    impl From<IntCtrl> for FieldSetValue {
        fn from(val: IntCtrl) -> Self {
            Self::IntCtrl(val)
        }
    }
    impl From<IfConf> for FieldSetValue {
        fn from(val: IfConf) -> Self {
            Self::IfConf(val)
        }
    }
    impl From<PwrCtrl> for FieldSetValue {
        fn from(val: PwrCtrl) -> Self {
            Self::PwrCtrl(val)
        }
    }
    impl From<Osr> for FieldSetValue {
        fn from(val: Osr) -> Self {
            Self::Osr(val)
        }
    }
    impl From<Odr> for FieldSetValue {
        fn from(val: Odr) -> Self {
            Self::Odr(val)
        }
    }
    impl From<Config> for FieldSetValue {
        fn from(val: Config) -> Self {
            Self::Config(val)
        }
    }
    impl From<CalibrationData> for FieldSetValue {
        fn from(val: CalibrationData) -> Self {
            Self::CalibrationData(val)
        }
    }
    impl From<CmdFieldsIn> for FieldSetValue {
        fn from(val: CmdFieldsIn) -> Self {
            Self::CmdFieldsIn(val)
        }
    }
}
/// CMD decoder status.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandStatus {
    /// Command execution in progress.
    InProgress = 0,
    /// Command decoder is ready to accept a new command.
    Ready = 1,
}
impl core::convert::TryFrom<u8> for CommandStatus {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::InProgress),
            1 => Ok(Self::Ready),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "CommandStatus",
                })
            }
        }
    }
}
impl From<CommandStatus> for u8 {
    fn from(val: CommandStatus) -> Self {
        match val {
            CommandStatus::InProgress => 0,
            CommandStatus::Ready => 1,
        }
    }
}
/// Whether FIFO is enabled or disabled.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoMode {
    /// FIFO is disabled.
    Disabled = 0,
    /// FIFO is enabled.
    Enabled = 1,
}
impl core::convert::TryFrom<u8> for FifoMode {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Disabled),
            1 => Ok(Self::Enabled),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "FifoMode",
                })
            }
        }
    }
}
impl From<FifoMode> for u8 {
    fn from(val: FifoMode) -> Self {
        match val {
            FifoMode::Disabled => 0,
            FifoMode::Enabled => 1,
        }
    }
}
/// Selects the data source for pressure and temperature data.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoDataSelect {
    /// Unfiltered data is written to FIFO (compensated or uncompensated).
    Unfiltered = 0,
    /// Filtered data is written to FIFO (compensated or uncompensated).
    Filtered = 1,
    Reserved(u8) = 2,
}
impl From<u8> for FifoDataSelect {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Unfiltered,
            1 => Self::Filtered,
            val => Self::Reserved(val),
        }
    }
}
impl From<FifoDataSelect> for u8 {
    fn from(val: FifoDataSelect) -> Self {
        match val {
            FifoDataSelect::Unfiltered => 0,
            FifoDataSelect::Filtered => 1,
            FifoDataSelect::Reserved(num) => num,
        }
    }
}
/// Configures the INT pin as push-pull or open-drain.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntOpenDrain {
    /// INT pin is push-pull.
    PushPull = 0,
    /// INT pin is open-drain.
    OpenDrain = 1,
}
impl core::convert::TryFrom<u8> for IntOpenDrain {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::PushPull),
            1 => Ok(Self::OpenDrain),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "IntOpenDrain",
                })
            }
        }
    }
}
impl From<IntOpenDrain> for u8 {
    fn from(val: IntOpenDrain) -> Self {
        match val {
            IntOpenDrain::PushPull => 0,
            IntOpenDrain::OpenDrain => 1,
        }
    }
}
/// Configures the INT pin as active high or active low.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IntLevel {
    /// INT pin is active low.
    ActiveLow = 0,
    /// INT pin is active high.
    ActiveHigh = 1,
}
impl core::convert::TryFrom<u8> for IntLevel {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::ActiveLow),
            1 => Ok(Self::ActiveHigh),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "IntLevel",
                })
            }
        }
    }
}
impl From<IntLevel> for u8 {
    fn from(val: IntLevel) -> Self {
        match val {
            IntLevel::ActiveLow => 0,
            IntLevel::ActiveHigh => 1,
        }
    }
}
/// Configure SPI Interface Mode for primary interface.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiMode {
    /// SPI 4-wire mode.
    Spi4 = 0,
    /// SPI 3-wire mode.
    Spi3 = 1,
}
impl core::convert::TryFrom<u8> for SpiMode {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Spi4),
            1 => Ok(Self::Spi3),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "SpiMode",
                })
            }
        }
    }
}
impl From<SpiMode> for u8 {
    fn from(val: SpiMode) -> Self {
        match val {
            SpiMode::Spi4 => 0,
            SpiMode::Spi3 => 1,
        }
    }
}
/// The timeout period for the I2C Watchdog timer.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2CWatchdogPeriod {
    /// I2C Watchdog timeout after 1.25 ms.
    Short = 0,
    /// I2C Watchdog timeout after 40 ms.
    Long = 1,
}
impl core::convert::TryFrom<u8> for I2CWatchdogPeriod {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Short),
            1 => Ok(Self::Long),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "I2CWatchdogPeriod",
                })
            }
        }
    }
}
impl From<I2CWatchdogPeriod> for u8 {
    fn from(val: I2CWatchdogPeriod) -> Self {
        match val {
            I2CWatchdogPeriod::Short => 0,
            I2CWatchdogPeriod::Long => 1,
        }
    }
}
/// Selects the measurement mode.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MeasurementMode {
    /// Sleep mode. No measurements are performed.
    Sleep = 0,
    /// Forced mode. One measurement is performed, then the device returns to sleep mode.
    Forced = 1,
    /// Forced mode. Equivalent to Forced (`0b01`).
    Forced2 = 2,
    /// Normal mode. Continuous measurements are performed at the selected sampling rate.
    Normal = 3,
}
impl core::convert::TryFrom<u8> for MeasurementMode {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Sleep),
            1 => Ok(Self::Forced),
            2 => Ok(Self::Forced2),
            3 => Ok(Self::Normal),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "MeasurementMode",
                })
            }
        }
    }
}
impl From<MeasurementMode> for u8 {
    fn from(val: MeasurementMode) -> Self {
        match val {
            MeasurementMode::Sleep => 0,
            MeasurementMode::Forced => 1,
            MeasurementMode::Forced2 => 2,
            MeasurementMode::Normal => 3,
        }
    }
}
/// Selects the oversampling rate for measurements.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Oversampling {
    /// No oversampling.
    X1 = 0,
    /// x2 oversampling.
    X2 = 1,
    /// x4 oversampling.
    X4 = 2,
    /// x8 oversampling.
    X8 = 3,
    /// x16 oversampling.
    X16 = 4,
    /// x32 oversampling.
    X32 = 5,
    Reserved(u8) = 6,
}
impl From<u8> for Oversampling {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::X1,
            1 => Self::X2,
            2 => Self::X4,
            3 => Self::X8,
            4 => Self::X16,
            5 => Self::X32,
            val => Self::Reserved(val),
        }
    }
}
impl From<Oversampling> for u8 {
    fn from(val: Oversampling) -> Self {
        match val {
            Oversampling::X1 => 0,
            Oversampling::X2 => 1,
            Oversampling::X4 => 2,
            Oversampling::X8 => 3,
            Oversampling::X16 => 4,
            Oversampling::X32 => 5,
            Oversampling::Reserved(num) => num,
        }
    }
}
/// The output data rate for pressure and temperature measurements: `2^value`.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OdrSel {
    /// 200 Hz, 5 ms sampling period, prescaler 1.
    Odr200 = 0,
    /// 100 Hz, 10 ms sampling period, prescaler 2.
    Odr100 = 1,
    /// 50 Hz, 20 ms sampling period, prescaler 4.
    Odr50 = 2,
    /// 25 Hz, 40 ms sampling period, prescaler 8.
    Odr25 = 3,
    /// 25/2 Hz, 80 ms sampling period, prescaler 16.
    Odr12P5 = 4,
    /// 25/4 Hz, 160 ms sampling period, prescaler 32.
    Odr6P25 = 5,
    /// 25/8 Hz, 320 ms sampling period, prescaler 64.
    Odr3P1 = 6,
    /// 25/16 Hz, 640 ms sampling period, prescaler 128.
    Odr1P5 = 7,
    /// 25/32 Hz, 1.280 s sampling period, prescaler 256.
    Odr0P78 = 8,
    /// 25/64 Hz, 2.560 s sampling period, prescaler 512.
    Odr0P39 = 9,
    /// 25/128 Hz, 5.120 s sampling period, prescaler 1024.
    Odr0P2 = 10,
    /// 25/256 Hz, 10.24 s sampling period, prescaler 2048.
    Odr0P1 = 11,
    /// 25/512 Hz, 20.48 s sampling period, prescaler 4096.
    Odr0P05 = 12,
    /// 25/1024 Hz, 40.96 s sampling period, prescaler 8192.
    Odr0P02 = 13,
    /// 25/2048 Hz, 81.92 s sampling period, prescaler 16384.
    Odr0P01 = 14,
    /// 25/4096 Hz, 163.84 s sampling period, prescaler 32768.
    Odr0P006 = 15,
    /// 25/8192 Hz, 327.68 s sampling period, prescaler 65536.
    Odr0P003 = 16,
    /// 25/16384 Hz, 655.36 s sampling period, prescaler 131072.
    Odr0P0015 = 17,
    Reserved(u8) = 18,
}
impl From<u8> for OdrSel {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Odr200,
            1 => Self::Odr100,
            2 => Self::Odr50,
            3 => Self::Odr25,
            4 => Self::Odr12P5,
            5 => Self::Odr6P25,
            6 => Self::Odr3P1,
            7 => Self::Odr1P5,
            8 => Self::Odr0P78,
            9 => Self::Odr0P39,
            10 => Self::Odr0P2,
            11 => Self::Odr0P1,
            12 => Self::Odr0P05,
            13 => Self::Odr0P02,
            14 => Self::Odr0P01,
            15 => Self::Odr0P006,
            16 => Self::Odr0P003,
            17 => Self::Odr0P0015,
            val => Self::Reserved(val),
        }
    }
}
impl From<OdrSel> for u8 {
    fn from(val: OdrSel) -> Self {
        match val {
            OdrSel::Odr200 => 0,
            OdrSel::Odr100 => 1,
            OdrSel::Odr50 => 2,
            OdrSel::Odr25 => 3,
            OdrSel::Odr12P5 => 4,
            OdrSel::Odr6P25 => 5,
            OdrSel::Odr3P1 => 6,
            OdrSel::Odr1P5 => 7,
            OdrSel::Odr0P78 => 8,
            OdrSel::Odr0P39 => 9,
            OdrSel::Odr0P2 => 10,
            OdrSel::Odr0P1 => 11,
            OdrSel::Odr0P05 => 12,
            OdrSel::Odr0P02 => 13,
            OdrSel::Odr0P01 => 14,
            OdrSel::Odr0P006 => 15,
            OdrSel::Odr0P003 => 16,
            OdrSel::Odr0P0015 => 17,
            OdrSel::Reserved(num) => num,
        }
    }
}
/// IIR filter coefficient selection.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IirFilterCoefficient {
    /// Filter coefficient is 0. Bypass mode.
    Coef0 = 0,
    /// Filter coefficient is 1.
    Coef1 = 1,
    /// Filter coefficient is 3.
    Coef3 = 2,
    /// Filter coefficient is 7.
    Coef7 = 3,
    /// Filter coefficient is 15.
    Coef15 = 4,
    /// Filter coefficient is 31.
    Coef31 = 5,
    /// Filter coefficient is 63.
    Coef63 = 6,
    /// Filter coefficient is 127.
    Coef127 = 7,
}
impl core::convert::TryFrom<u8> for IirFilterCoefficient {
    type Error = ::device_driver::ConversionError<u8>;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        match val {
            0 => Ok(Self::Coef0),
            1 => Ok(Self::Coef1),
            2 => Ok(Self::Coef3),
            3 => Ok(Self::Coef7),
            4 => Ok(Self::Coef15),
            5 => Ok(Self::Coef31),
            6 => Ok(Self::Coef63),
            7 => Ok(Self::Coef127),
            val => {
                Err(::device_driver::ConversionError {
                    source: val,
                    target: "IirFilterCoefficient",
                })
            }
        }
    }
}
impl From<IirFilterCoefficient> for u8 {
    fn from(val: IirFilterCoefficient) -> Self {
        match val {
            IirFilterCoefficient::Coef0 => 0,
            IirFilterCoefficient::Coef1 => 1,
            IirFilterCoefficient::Coef3 => 2,
            IirFilterCoefficient::Coef7 => 3,
            IirFilterCoefficient::Coef15 => 4,
            IirFilterCoefficient::Coef31 => 5,
            IirFilterCoefficient::Coef63 => 6,
            IirFilterCoefficient::Coef127 => 7,
        }
    }
}
/// Command to execute.
#[repr(u8)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// No operation.
    Nop = 0,
    /// Clears all data in the FIFO. Does not change FIFO_CONFIG registers.
    FifoFlush = 176,
    /// Triggers a reset. All user configuration settings are overwritten with their default state.
    SoftReset = 182,
    Reserved(u8) = 183,
}
impl From<u8> for Command {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Nop,
            176 => Self::FifoFlush,
            182 => Self::SoftReset,
            val => Self::Reserved(val),
        }
    }
}
impl From<Command> for u8 {
    fn from(val: Command) -> Self {
        match val {
            Command::Nop => 0,
            Command::FifoFlush => 176,
            Command::SoftReset => 182,
            Command::Reserved(num) => num,
        }
    }
}
