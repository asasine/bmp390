// This code was generated using device-driver `2.1.0` (),
// a tool distributed under MIT OR Apache-2.0 by Dion Dokter <dev@diondokter.nl>
// 
// For more information about device-driver, visit the website: https://device-driver.com

/// Root block of the Device driver
#[derive(Debug)]
pub struct Device<I> {
    interface: I,
    #[doc(hidden)]
    #[allow(unused)]
    base_address: u8,
}
impl<I> Device<I> {
    /// Create a new instance of the device
    pub const fn new(interface: I) -> Self {
        Self { interface, base_address: 0 }
    }
    /// Drop the driver instance and reclaim the interface
    pub fn free(self) -> I {
        self.interface
    }
    /// The Chip ID register contains the chip identification code.
    ///
    /// Register operation:
    /// - Address: `0`
    /// - Reset value: `0x60`
    #[doc(alias = "ChipId")]
    pub fn chip_id(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        ChipId,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 0;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || ChipId::from([96]),
        )
    }
    /// The Revision ID register contains the mask revision of the ASIC.
    ///
    /// Register operation:
    /// - Address: `1`
    /// - Reset value: `0x01`
    #[doc(alias = "RevId")]
    pub fn rev_id(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, RevId, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 1;
        ::device_driver::RegisterOperation::new(self, address as u8, || RevId::from([1]))
    }
    /// Sensor error conditions.
    ///
    /// Register operation:
    /// - Address: `2`
    /// - Reset value: `0`
    #[doc(alias = "ErrReg")]
    pub fn err_reg(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        ErrReg,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 2;
        ::device_driver::RegisterOperation::new(self, address as u8, ErrReg::default)
    }
    /// Sensor status flags.
    ///
    /// Register operation:
    /// - Address: `3`
    /// - Reset value: `0`
    #[doc(alias = "Status")]
    pub fn status(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        Status,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 3;
        ::device_driver::RegisterOperation::new(self, address as u8, Status::default)
    }
    /// 24-bit pressure data, split and stored in three consecutive registers.
    ///
    /// Register operation:
    /// - Address: `4`
    /// - Reset value: `0x800000`
    #[doc(alias = "PressureData")]
    pub fn pressure_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        PressureData,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 4;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || PressureData::from([0, 0, 128]),
        )
    }
    /// 24-bit temperature data, split and stored in three consecutive registers.
    ///
    /// Register operation:
    /// - Address: `7`
    /// - Reset value: `0x800000`
    #[doc(alias = "TemperatureData")]
    pub fn temperature_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        TemperatureData,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 7;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || TemperatureData::from([0, 0, 128]),
        )
    }
    /// 24-bit sensor time data, split and stored in three consecutive registers.
    ///
    /// Register operation:
    /// - Address: `12`
    /// - Reset value: `0`
    #[doc(alias = "SensorTime")]
    pub fn sensor_time(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        SensorTime,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 12;
        ::device_driver::RegisterOperation::new(self, address as u8, SensorTime::default)
    }
    /// Pressure and temperature data, split and stored in six consecutive registers. Reading in one burst read ensures that the pressure and temperature data are from the same measurement sampling.
    ///
    /// Register operation:
    /// - Address: `4`
    /// - Reset value: `0x800000800000`
    #[doc(alias = "Data")]
    pub fn data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Data, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 4;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || Data::from([0, 0, 128, 0, 0, 128]),
        )
    }
    /// Sensor status flags.
    ///
    /// Register operation:
    /// - Address: `16`
    /// - Reset value: `0x01`
    #[doc(alias = "Event")]
    pub fn event(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Event, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 16;
        ::device_driver::RegisterOperation::new(self, address as u8, || Event::from([1]))
    }
    /// Interrupt status. Cleared after reading.
    ///
    /// Register operation:
    /// - Address: `17`
    /// - Reset value: `0`
    #[doc(alias = "IntStatus")]
    pub fn int_status(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        IntStatus,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 17;
        ::device_driver::RegisterOperation::new(self, address as u8, IntStatus::default)
    }
    /// Indicates the current fill level of the FIFO buffer.
    ///
    /// Register operation:
    /// - Address: `18`
    /// - Reset value: `0`
    #[doc(alias = "FifoLength")]
    pub fn fifo_length(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        FifoLength,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 18;
        ::device_driver::RegisterOperation::new(self, address as u8, FifoLength::default)
    }
    /// FIFO data output.
    ///
    /// Register operation:
    /// - Address: `20`
    /// - Reset value: `0`
    #[doc(alias = "FifoData")]
    pub fn fifo_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        FifoData,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 20;
        ::device_driver::RegisterOperation::new(self, address as u8, FifoData::default)
    }
    /// The FIFO watermark level.
    ///
    /// Register operation:
    /// - Address: `21`
    /// - Reset value: `0x001`
    #[doc(alias = "FifoWatermark")]
    pub fn fifo_watermark(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        FifoWatermark,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 21;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || FifoWatermark::from([1, 0]),
        )
    }
    /// FIFO frame content configuration.
    ///
    /// Register operation:
    /// - Address: `23`
    /// - Reset value: `0x0202`
    #[doc(alias = "FifoConfig")]
    pub fn fifo_config(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        FifoConfig,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 23;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || FifoConfig::from([2, 2]),
        )
    }
    /// Interrupt configuration, controlling IntStatus register and the INT pin.
    ///
    /// Register operation:
    /// - Address: `25`
    /// - Reset value: `0x02`
    #[doc(alias = "IntCtrl")]
    pub fn int_ctrl(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        IntCtrl,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 25;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            || IntCtrl::from([2]),
        )
    }
    /// Serial interface settings.
    ///
    /// Register operation:
    /// - Address: `26`
    /// - Reset value: `0`
    #[doc(alias = "IfConf")]
    pub fn if_conf(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        IfConf,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 26;
        ::device_driver::RegisterOperation::new(self, address as u8, IfConf::default)
    }
    /// Enables or disables pressure and temperature measurements and the measurement mode.
    ///
    /// Register operation:
    /// - Address: `27`
    /// - Reset value: `0`
    #[doc(alias = "PwrCtrl")]
    pub fn pwr_ctrl(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        PwrCtrl,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 27;
        ::device_driver::RegisterOperation::new(self, address as u8, PwrCtrl::default)
    }
    /// Controls the oversampling settings for pressure and temperature measurements.
    ///
    /// Register operation:
    /// - Address: `28`
    /// - Reset value: `0x02`
    #[doc(alias = "Osr")]
    pub fn osr(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Osr, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 28;
        ::device_driver::RegisterOperation::new(self, address as u8, || Osr::from([2]))
    }
    /// Controls the output data rate by means of setting the subdivision/subsampling.
    ///
    /// Register operation:
    /// - Address: `29`
    /// - Reset value: `0`
    #[doc(alias = "Odr")]
    pub fn odr(
        &mut self,
    ) -> ::device_driver::RegisterOperation<'_, Self, Odr, u8, ::device_driver::RW, ()>
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 29;
        ::device_driver::RegisterOperation::new(self, address as u8, Odr::default)
    }
    /// IIR filter coefficients.
    ///
    /// Register operation:
    /// - Address: `31`
    /// - Reset value: `0`
    #[doc(alias = "Config")]
    pub fn config(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        Config,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 31;
        ::device_driver::RegisterOperation::new(self, address as u8, Config::default)
    }
    /// Calibration data for pressure and temperature compensation.
    ///
    /// Register operation:
    /// - Address: `48`
    /// - Reset value: `0`
    #[doc(alias = "CalibrationData")]
    pub fn calibration_data(
        &mut self,
    ) -> ::device_driver::RegisterOperation<
        '_,
        Self,
        CalibrationData,
        u8,
        ::device_driver::RW,
        (),
    >
    where
        I: ::device_driver::RegisterInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 48;
        ::device_driver::RegisterOperation::new(
            self,
            address as u8,
            CalibrationData::default,
        )
    }
    /// Command operation:
    /// - Address: `126`
    #[doc(alias = "Cmd")]
    pub fn cmd(
        &mut self,
    ) -> ::device_driver::CommandOperation<'_, Self, u8, CmdFieldsIn, (), ()>
    where
        I: ::device_driver::CommandInterfaceBase<AddressType = u8>,
    {
        let address = self.base_address + 126;
        ::device_driver::CommandOperation::new(self, address as u8)
    }
}
impl<I> ::device_driver::Block for Device<I> {
    type Interface = I;
    type RegisterAddressType = u8;
    type CommandAddressType = u8;
    type BufferAddressType = u8;
    type RegisterAddressMode = ::device_driver::MappedAddressMode;
    fn interface(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct CmdFieldsIn {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for CmdFieldsIn {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl CmdFieldsIn {
    /// `7:0` - Read the `cmd` field.
    ///
    /// Command to execute.
    #[must_use]
    pub fn cmd(&self) -> Command {
        let start = 0;
        let end = 7;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw.into()
    }
    /// `7:0` - Set the `cmd` field.
    ///
    /// Command to execute.
    pub fn set_cmd(&mut self, value: Command) {
        let start = 0;
        let end = 7;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for CmdFieldsIn {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct CalibrationData {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 22],
}
unsafe impl ::device_driver::Fieldset for CalibrationData {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 22] };
}
impl CalibrationData {
    /// `23:8` - Read the `nvm_par_t_1` field.
    ///
    #[doc(alias = "nvm_par_t1")]
    #[must_use]
    pub fn nvm_par_t_1(&self) -> u16 {
        let start = 8;
        let end = 23;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `39:24` - Read the `nvm_par_t_2` field.
    ///
    #[doc(alias = "nvm_par_t2")]
    #[must_use]
    pub fn nvm_par_t_2(&self) -> u16 {
        let start = 24;
        let end = 39;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `47:40` - Read the `nvm_par_t_3` field.
    ///
    #[doc(alias = "nvm_par_t3")]
    #[must_use]
    pub fn nvm_par_t_3(&self) -> i8 {
        let start = 40;
        let end = 47;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `63:48` - Read the `nvm_par_p_1` field.
    ///
    #[doc(alias = "nvm_par_p1")]
    #[must_use]
    pub fn nvm_par_p_1(&self) -> i16 {
        let start = 48;
        let end = 63;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `79:64` - Read the `nvm_par_p_2` field.
    ///
    #[doc(alias = "nvm_par_p2")]
    #[must_use]
    pub fn nvm_par_p_2(&self) -> i16 {
        let start = 64;
        let end = 79;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `87:80` - Read the `nvm_par_p_3` field.
    ///
    #[doc(alias = "nvm_par_p3")]
    #[must_use]
    pub fn nvm_par_p_3(&self) -> i8 {
        let start = 80;
        let end = 87;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `95:88` - Read the `nvm_par_p_4` field.
    ///
    #[doc(alias = "nvm_par_p4")]
    #[must_use]
    pub fn nvm_par_p_4(&self) -> i8 {
        let start = 88;
        let end = 95;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `111:96` - Read the `nvm_par_p_5` field.
    ///
    #[doc(alias = "nvm_par_p5")]
    #[must_use]
    pub fn nvm_par_p_5(&self) -> u16 {
        let start = 96;
        let end = 111;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `127:112` - Read the `nvm_par_p_6` field.
    ///
    #[doc(alias = "nvm_par_p6")]
    #[must_use]
    pub fn nvm_par_p_6(&self) -> u16 {
        let start = 112;
        let end = 127;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `135:128` - Read the `nvm_par_p_7` field.
    ///
    #[doc(alias = "nvm_par_p7")]
    #[must_use]
    pub fn nvm_par_p_7(&self) -> i8 {
        let start = 128;
        let end = 135;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `143:136` - Read the `nvm_par_p_8` field.
    ///
    #[doc(alias = "nvm_par_p8")]
    #[must_use]
    pub fn nvm_par_p_8(&self) -> i8 {
        let start = 136;
        let end = 143;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `159:144` - Read the `nvm_par_p_9` field.
    ///
    #[doc(alias = "nvm_par_p9")]
    #[must_use]
    pub fn nvm_par_p_9(&self) -> i16 {
        let start = 144;
        let end = 159;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `167:160` - Read the `nvm_par_p_10` field.
    ///
    #[doc(alias = "nvm_par_p10")]
    #[must_use]
    pub fn nvm_par_p_10(&self) -> i8 {
        let start = 160;
        let end = 167;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `175:168` - Read the `nvm_par_p_11` field.
    ///
    #[doc(alias = "nvm_par_p11")]
    #[must_use]
    pub fn nvm_par_p_11(&self) -> i8 {
        let start = 168;
        let end = 175;
        let raw = unsafe {
            ::device_driver::ops::load::<
                i8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `23:8` - Set the `nvm_par_t_1` field.
    ///
    #[doc(alias = "nvm_par_t1")]
    pub fn set_nvm_par_t_1(&mut self, value: u16) {
        let start = 8;
        let end = 23;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `39:24` - Set the `nvm_par_t_2` field.
    ///
    #[doc(alias = "nvm_par_t2")]
    pub fn set_nvm_par_t_2(&mut self, value: u16) {
        let start = 24;
        let end = 39;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `47:40` - Set the `nvm_par_t_3` field.
    ///
    #[doc(alias = "nvm_par_t3")]
    pub fn set_nvm_par_t_3(&mut self, value: i8) {
        let start = 40;
        let end = 47;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `63:48` - Set the `nvm_par_p_1` field.
    ///
    #[doc(alias = "nvm_par_p1")]
    pub fn set_nvm_par_p_1(&mut self, value: i16) {
        let start = 48;
        let end = 63;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `79:64` - Set the `nvm_par_p_2` field.
    ///
    #[doc(alias = "nvm_par_p2")]
    pub fn set_nvm_par_p_2(&mut self, value: i16) {
        let start = 64;
        let end = 79;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `87:80` - Set the `nvm_par_p_3` field.
    ///
    #[doc(alias = "nvm_par_p3")]
    pub fn set_nvm_par_p_3(&mut self, value: i8) {
        let start = 80;
        let end = 87;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `95:88` - Set the `nvm_par_p_4` field.
    ///
    #[doc(alias = "nvm_par_p4")]
    pub fn set_nvm_par_p_4(&mut self, value: i8) {
        let start = 88;
        let end = 95;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `111:96` - Set the `nvm_par_p_5` field.
    ///
    #[doc(alias = "nvm_par_p5")]
    pub fn set_nvm_par_p_5(&mut self, value: u16) {
        let start = 96;
        let end = 111;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `127:112` - Set the `nvm_par_p_6` field.
    ///
    #[doc(alias = "nvm_par_p6")]
    pub fn set_nvm_par_p_6(&mut self, value: u16) {
        let start = 112;
        let end = 127;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `135:128` - Set the `nvm_par_p_7` field.
    ///
    #[doc(alias = "nvm_par_p7")]
    pub fn set_nvm_par_p_7(&mut self, value: i8) {
        let start = 128;
        let end = 135;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `143:136` - Set the `nvm_par_p_8` field.
    ///
    #[doc(alias = "nvm_par_p8")]
    pub fn set_nvm_par_p_8(&mut self, value: i8) {
        let start = 136;
        let end = 143;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `159:144` - Set the `nvm_par_p_9` field.
    ///
    #[doc(alias = "nvm_par_p9")]
    pub fn set_nvm_par_p_9(&mut self, value: i16) {
        let start = 144;
        let end = 159;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `167:160` - Set the `nvm_par_p_10` field.
    ///
    #[doc(alias = "nvm_par_p10")]
    pub fn set_nvm_par_p_10(&mut self, value: i8) {
        let start = 160;
        let end = 167;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `175:168` - Set the `nvm_par_p_11` field.
    ///
    #[doc(alias = "nvm_par_p11")]
    pub fn set_nvm_par_p_11(&mut self, value: i8) {
        let start = 168;
        let end = 175;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                i8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for CalibrationData {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[repr(transparent)]
pub struct Config {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for Config {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl Config {
    /// `3:1` - Read the `iir_filter` field.
    ///
    /// Filter coefficient for IIR filter.
    #[must_use]
    pub fn iir_filter(&self) -> IirFilterCoefficient {
        let start = 1;
        let end = 3;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `3:1` - Set the `iir_filter` field.
    ///
    /// Filter coefficient for IIR filter.
    pub fn set_iir_filter(&mut self, value: IirFilterCoefficient) {
        let start = 1;
        let end = 3;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Config {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Odr {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for Odr {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl Odr {
    /// `4:0` - Read the `odr_sel` field.
    ///
    /// Subdivision factor for pressure and temperature measurement is `2^value`.
    #[must_use]
    pub fn odr_sel(&self) -> OdrSel {
        let start = 0;
        let end = 4;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw.into()
    }
    /// `4:0` - Set the `odr_sel` field.
    ///
    /// Subdivision factor for pressure and temperature measurement is `2^value`.
    pub fn set_odr_sel(&mut self, value: OdrSel) {
        let start = 0;
        let end = 4;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Odr {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Osr {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for Osr {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl Osr {
    /// `2:0` - Read the `pressure` field.
    ///
    /// Selects the oversampling rate for pressure measurements.
    #[must_use]
    pub fn pressure(&self) -> Oversampling {
        let start = 0;
        let end = 2;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw.into()
    }
    /// `5:3` - Read the `temperature` field.
    ///
    /// Selects the oversampling rate for temperature measurements.
    #[must_use]
    pub fn temperature(&self) -> Oversampling {
        let start = 3;
        let end = 5;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw.into()
    }
    /// `2:0` - Set the `pressure` field.
    ///
    /// Selects the oversampling rate for pressure measurements.
    pub fn set_pressure(&mut self, value: Oversampling) {
        let start = 0;
        let end = 2;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `5:3` - Set the `temperature` field.
    ///
    /// Selects the oversampling rate for temperature measurements.
    pub fn set_temperature(&mut self, value: Oversampling) {
        let start = 3;
        let end = 5;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Osr {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct PwrCtrl {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for PwrCtrl {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl PwrCtrl {
    /// `bit 0` - Read the `pressure_enable` field.
    ///
    /// Whether to enable pressure measurements. True to enable.
    #[must_use]
    pub fn pressure_enable(&self) -> bool {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 1` - Read the `temperature_enable` field.
    ///
    /// Whether to enable temperature measurements. True to enable.
    #[must_use]
    pub fn temperature_enable(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `5:4` - Read the `mode` field.
    ///
    /// Selects the measurement mode.
    #[must_use]
    pub fn mode(&self) -> MeasurementMode {
        let start = 4;
        let end = 5;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 0` - Set the `pressure_enable` field.
    ///
    /// Whether to enable pressure measurements. True to enable.
    pub fn set_pressure_enable(&mut self, value: bool) {
        let start = 0;
        let end = 0;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `temperature_enable` field.
    ///
    /// Whether to enable temperature measurements. True to enable.
    pub fn set_temperature_enable(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `5:4` - Set the `mode` field.
    ///
    /// Selects the measurement mode.
    pub fn set_mode(&mut self, value: MeasurementMode) {
        let start = 4;
        let end = 5;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for PwrCtrl {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
        defmt::write!(f, "temperature_enable: {=bool}, ", & self.temperature_enable());
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct IfConf {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for IfConf {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl IfConf {
    /// `bit 0` - Read the `spi` field.
    ///
    /// Configure SPI Interface Mode for primary interface.
    #[must_use]
    pub fn spi(&self) -> SpiMode {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 1` - Read the `i_2_c_wdt_en` field.
    ///
    /// Whether to enable the I2C Watchdog timer, backed by NVM. True to enable.
    #[doc(alias = "i2c_wdt_en")]
    #[must_use]
    pub fn i_2_c_wdt_en(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 2` - Read the `i_2_c_wdt_sel` field.
    ///
    /// Selects the timer period for the I2C Watchdog, backed by NVM.
    #[doc(alias = "i2c_wdt_sel")]
    #[must_use]
    pub fn i_2_c_wdt_sel(&self) -> I2CWatchdogPeriod {
        let start = 2;
        let end = 2;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 0` - Set the `spi` field.
    ///
    /// Configure SPI Interface Mode for primary interface.
    pub fn set_spi(&mut self, value: SpiMode) {
        let start = 0;
        let end = 0;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `i_2_c_wdt_en` field.
    ///
    /// Whether to enable the I2C Watchdog timer, backed by NVM. True to enable.
    #[doc(alias = "i2c_wdt_en")]
    pub fn set_i_2_c_wdt_en(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 2` - Set the `i_2_c_wdt_sel` field.
    ///
    /// Selects the timer period for the I2C Watchdog, backed by NVM.
    #[doc(alias = "i2c_wdt_sel")]
    pub fn set_i_2_c_wdt_sel(&mut self, value: I2CWatchdogPeriod) {
        let start = 2;
        let end = 2;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for IfConf {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct IntCtrl {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for IntCtrl {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl IntCtrl {
    /// `bit 0` - Read the `int_open_drain` field.
    ///
    /// Configures the INT pin as push-pull or open-drain.
    #[must_use]
    pub fn int_open_drain(&self) -> IntOpenDrain {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 1` - Read the `int_level` field.
    ///
    /// Configures the INT pin as active high or active low.
    #[must_use]
    pub fn int_level(&self) -> IntLevel {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 2` - Read the `int_latch` field.
    ///
    /// Configures the INT pin as latched or pulsed. True for latched.
    #[must_use]
    pub fn int_latch(&self) -> bool {
        let start = 2;
        let end = 2;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 3` - Read the `fifo_watermark_int_enable` field.
    ///
    /// Whether to interrupt when the FIFO watermark is reached. True to interrupt when the FIFO watermark is reached.
    #[must_use]
    pub fn fifo_watermark_int_enable(&self) -> bool {
        let start = 3;
        let end = 3;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 4` - Read the `fifo_full_int_enable` field.
    ///
    /// Whether to interrupt when the FIFO is full. True to interrupt when the FIFO is full.
    #[must_use]
    pub fn fifo_full_int_enable(&self) -> bool {
        let start = 4;
        let end = 4;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 5` - Read the `int_ds` field.
    ///
    #[must_use]
    pub fn int_ds(&self) -> bool {
        let start = 5;
        let end = 5;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 6` - Read the `data_ready_int_enable` field.
    ///
    /// Whether to interrupt when data is ready for pressure and temperature. True to interrupt when data is ready for pressure and temperature.
    #[must_use]
    pub fn data_ready_int_enable(&self) -> bool {
        let start = 6;
        let end = 6;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 0` - Set the `int_open_drain` field.
    ///
    /// Configures the INT pin as push-pull or open-drain.
    pub fn set_int_open_drain(&mut self, value: IntOpenDrain) {
        let start = 0;
        let end = 0;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `int_level` field.
    ///
    /// Configures the INT pin as active high or active low.
    pub fn set_int_level(&mut self, value: IntLevel) {
        let start = 1;
        let end = 1;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 2` - Set the `int_latch` field.
    ///
    /// Configures the INT pin as latched or pulsed. True for latched.
    pub fn set_int_latch(&mut self, value: bool) {
        let start = 2;
        let end = 2;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 3` - Set the `fifo_watermark_int_enable` field.
    ///
    /// Whether to interrupt when the FIFO watermark is reached. True to interrupt when the FIFO watermark is reached.
    pub fn set_fifo_watermark_int_enable(&mut self, value: bool) {
        let start = 3;
        let end = 3;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 4` - Set the `fifo_full_int_enable` field.
    ///
    /// Whether to interrupt when the FIFO is full. True to interrupt when the FIFO is full.
    pub fn set_fifo_full_int_enable(&mut self, value: bool) {
        let start = 4;
        let end = 4;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 5` - Set the `int_ds` field.
    ///
    pub fn set_int_ds(&mut self, value: bool) {
        let start = 5;
        let end = 5;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 6` - Set the `data_ready_int_enable` field.
    ///
    /// Whether to interrupt when data is ready for pressure and temperature. True to interrupt when data is ready for pressure and temperature.
    pub fn set_data_ready_int_enable(&mut self, value: bool) {
        let start = 6;
        let end = 6;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for IntCtrl {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
            f, "fifo_watermark_int_enable: {=bool}, ", & self.fifo_watermark_int_enable()
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct FifoConfig {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for FifoConfig {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl FifoConfig {
    /// `bit 0` - Read the `mode` field.
    ///
    /// Enables or disables the FIFO.
    #[must_use]
    pub fn mode(&self) -> FifoMode {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 1` - Read the `stop_on_full` field.
    ///
    /// Whether to stop writing samples into FIFO when it is full. True to stop writing samples into FIFO when it is full.
    #[must_use]
    pub fn stop_on_full(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 2` - Read the `time_enable` field.
    ///
    /// Whether to return sensortime frame after the last valid data frame. True to return sensortime frame after the last valid data frame.
    #[must_use]
    pub fn time_enable(&self) -> bool {
        let start = 2;
        let end = 2;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 3` - Read the `pressure_enable` field.
    ///
    /// Whether to store pressure data in FIFO. True to store pressure data in FIFO.
    #[must_use]
    pub fn pressure_enable(&self) -> bool {
        let start = 3;
        let end = 3;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 4` - Read the `temperature_enable` field.
    ///
    /// Whether to store temperature data in FIFO. True to store temperature data in FIFO.
    #[must_use]
    pub fn temperature_enable(&self) -> bool {
        let start = 4;
        let end = 4;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `10:8` - Read the `subsampling` field.
    ///
    /// FIFO downsampling selection for pressure and temperature data. Factor is `2^sampling`.
    #[must_use]
    pub fn subsampling(&self) -> u8 {
        let start = 8;
        let end = 10;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `12:11` - Read the `data_select` field.
    ///
    /// Selects the data source for pressure and temperature data.
    #[must_use]
    pub fn data_select(&self) -> FifoDataSelect {
        let start = 11;
        let end = 12;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw.into()
    }
    /// `bit 0` - Set the `mode` field.
    ///
    /// Enables or disables the FIFO.
    pub fn set_mode(&mut self, value: FifoMode) {
        let start = 0;
        let end = 0;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `stop_on_full` field.
    ///
    /// Whether to stop writing samples into FIFO when it is full. True to stop writing samples into FIFO when it is full.
    pub fn set_stop_on_full(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 2` - Set the `time_enable` field.
    ///
    /// Whether to return sensortime frame after the last valid data frame. True to return sensortime frame after the last valid data frame.
    pub fn set_time_enable(&mut self, value: bool) {
        let start = 2;
        let end = 2;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 3` - Set the `pressure_enable` field.
    ///
    /// Whether to store pressure data in FIFO. True to store pressure data in FIFO.
    pub fn set_pressure_enable(&mut self, value: bool) {
        let start = 3;
        let end = 3;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 4` - Set the `temperature_enable` field.
    ///
    /// Whether to store temperature data in FIFO. True to store temperature data in FIFO.
    pub fn set_temperature_enable(&mut self, value: bool) {
        let start = 4;
        let end = 4;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `10:8` - Set the `subsampling` field.
    ///
    /// FIFO downsampling selection for pressure and temperature data. Factor is `2^sampling`.
    pub fn set_subsampling(&mut self, value: u8) {
        let start = 8;
        let end = 10;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `12:11` - Set the `data_select` field.
    ///
    /// Selects the data source for pressure and temperature data.
    pub fn set_data_select(&mut self, value: FifoDataSelect) {
        let start = 11;
        let end = 12;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for FifoConfig {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
        defmt::write!(f, "temperature_enable: {=bool}, ", & self.temperature_enable());
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct FifoWatermark {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for FifoWatermark {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl FifoWatermark {}
impl Default for FifoWatermark {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct FifoData {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for FifoData {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl FifoData {}
impl Default for FifoData {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct FifoLength {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 2],
}
unsafe impl ::device_driver::Fieldset for FifoLength {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 2] };
}
impl FifoLength {
    /// `8:0` - Read the `value` field.
    ///
    /// The current fill level of the FIFO buffer.
    #[must_use]
    pub fn value(&self) -> u16 {
        let start = 0;
        let end = 8;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u16,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `8:0` - Set the `value` field.
    ///
    /// The current fill level of the FIFO buffer.
    pub fn set_value(&mut self, value: u16) {
        let start = 0;
        let end = 8;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u16,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for FifoLength {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct IntStatus {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for IntStatus {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl IntStatus {
    /// `bit 0` - Read the `fifo_full_watermark` field.
    ///
    /// The FIFO Watermark has been reached.
    #[must_use]
    pub fn fifo_full_watermark(&self) -> bool {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 1` - Read the `fifo_full` field.
    ///
    /// The FIFO is full.
    #[must_use]
    pub fn fifo_full(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 3` - Read the `data_ready` field.
    ///
    /// Data ready for pressure and temperature.
    #[must_use]
    pub fn data_ready(&self) -> bool {
        let start = 3;
        let end = 3;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 0` - Set the `fifo_full_watermark` field.
    ///
    /// The FIFO Watermark has been reached.
    pub fn set_fifo_full_watermark(&mut self, value: bool) {
        let start = 0;
        let end = 0;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `fifo_full` field.
    ///
    /// The FIFO is full.
    pub fn set_fifo_full(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 3` - Set the `data_ready` field.
    ///
    /// Data ready for pressure and temperature.
    pub fn set_data_ready(&mut self, value: bool) {
        let start = 3;
        let end = 3;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for IntStatus {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
        defmt::write!(f, "fifo_full_watermark: {=bool}, ", & self.fifo_full_watermark());
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Event {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for Event {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl Event {
    /// `bit 0` - Read the `por_detected` field.
    ///
    /// True after device power up or soft reset. Cleared on read.
    #[must_use]
    pub fn por_detected(&self) -> bool {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 1` - Read the `itf_act_pt` field.
    ///
    /// True when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read.
    #[must_use]
    pub fn itf_act_pt(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 0` - Set the `por_detected` field.
    ///
    /// True after device power up or soft reset. Cleared on read.
    pub fn set_por_detected(&mut self, value: bool) {
        let start = 0;
        let end = 0;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `itf_act_pt` field.
    ///
    /// True when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read.
    pub fn set_itf_act_pt(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Event {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Data {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 6],
}
unsafe impl ::device_driver::Fieldset for Data {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 6] };
}
impl Data {
    /// `23:0` - Read the `pressure` field.
    ///
    /// 24-bit pressure data.
    #[must_use]
    pub fn pressure(&self) -> u32 {
        let start = 0;
        let end = 23;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u32,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `47:24` - Read the `temperature` field.
    ///
    /// 24-bit temperature data.
    #[must_use]
    pub fn temperature(&self) -> u32 {
        let start = 24;
        let end = 47;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u32,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `23:0` - Set the `pressure` field.
    ///
    /// 24-bit pressure data.
    pub fn set_pressure(&mut self, value: u32) {
        let start = 0;
        let end = 23;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u32,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `47:24` - Set the `temperature` field.
    ///
    /// 24-bit temperature data.
    pub fn set_temperature(&mut self, value: u32) {
        let start = 24;
        let end = 47;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u32,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Data {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct SensorTime {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 3],
}
unsafe impl ::device_driver::Fieldset for SensorTime {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 3] };
}
impl SensorTime {
    /// `23:0` - Read the `value` field.
    ///
    /// 24-bit sensor time data.
    #[must_use]
    pub fn value(&self) -> u32 {
        let start = 0;
        let end = 23;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u32,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `23:0` - Set the `value` field.
    ///
    /// 24-bit sensor time data.
    pub fn set_value(&mut self, value: u32) {
        let start = 0;
        let end = 23;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u32,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for SensorTime {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct TemperatureData {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 3],
}
unsafe impl ::device_driver::Fieldset for TemperatureData {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 3] };
}
impl TemperatureData {
    /// `23:0` - Read the `value` field.
    ///
    /// 24-bit temperature data.
    #[must_use]
    pub fn value(&self) -> u32 {
        let start = 0;
        let end = 23;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u32,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `23:0` - Set the `value` field.
    ///
    /// 24-bit temperature data.
    pub fn set_value(&mut self, value: u32) {
        let start = 0;
        let end = 23;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u32,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for TemperatureData {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct PressureData {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 3],
}
unsafe impl ::device_driver::Fieldset for PressureData {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 3] };
}
impl PressureData {
    /// `23:0` - Read the `value` field.
    ///
    /// 24-bit pressure data.
    #[must_use]
    pub fn value(&self) -> u32 {
        let start = 0;
        let end = 23;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u32,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `23:0` - Set the `value` field.
    ///
    /// 24-bit pressure data.
    pub fn set_value(&mut self, value: u32) {
        let start = 0;
        let end = 23;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u32,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for PressureData {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct Status {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for Status {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl Status {
    /// `bit 4` - Read the `command_ready` field.
    ///
    /// CMD decoder status.
    #[must_use]
    pub fn command_ready(&self) -> CommandStatus {
        let start = 4;
        let end = 4;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        unsafe { raw.try_into().unwrap_unchecked() }
    }
    /// `bit 5` - Read the `data_ready_pressure` field.
    ///
    /// Data ready for pressure. Reset when one pressure DATA register is read out.
    #[must_use]
    pub fn data_ready_pressure(&self) -> bool {
        let start = 5;
        let end = 5;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 6` - Read the `data_ready_temperature` field.
    ///
    /// Data ready for temperature. Reset when the temperature DATA register is read out.
    #[must_use]
    pub fn data_ready_temperature(&self) -> bool {
        let start = 6;
        let end = 6;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 4` - Set the `command_ready` field.
    ///
    /// CMD decoder status.
    pub fn set_command_ready(&mut self, value: CommandStatus) {
        let start = 4;
        let end = 4;
        let raw = value.into();
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 5` - Set the `data_ready_pressure` field.
    ///
    /// Data ready for pressure. Reset when one pressure DATA register is read out.
    pub fn set_data_ready_pressure(&mut self, value: bool) {
        let start = 5;
        let end = 5;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 6` - Set the `data_ready_temperature` field.
    ///
    /// Data ready for temperature. Reset when the temperature DATA register is read out.
    pub fn set_data_ready_temperature(&mut self, value: bool) {
        let start = 6;
        let end = 6;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for Status {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
        defmt::write!(f, "data_ready_pressure: {=bool}, ", & self.data_ready_pressure());
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct ErrReg {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for ErrReg {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl ErrReg {
    /// `bit 0` - Read the `fatal_err` field.
    ///
    /// Fatal error.
    #[must_use]
    pub fn fatal_err(&self) -> bool {
        let start = 0;
        let end = 0;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 1` - Read the `cmd_err` field.
    ///
    /// Command execution failed. Cleared on read.
    #[must_use]
    pub fn cmd_err(&self) -> bool {
        let start = 1;
        let end = 1;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 2` - Read the `conf_err` field.
    ///
    /// Sensor configuration error detected (only working in normal mode). Cleared on read.
    #[must_use]
    pub fn conf_err(&self) -> bool {
        let start = 2;
        let end = 2;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw > 0
    }
    /// `bit 0` - Set the `fatal_err` field.
    ///
    /// Fatal error.
    pub fn set_fatal_err(&mut self, value: bool) {
        let start = 0;
        let end = 0;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 1` - Set the `cmd_err` field.
    ///
    /// Command execution failed. Cleared on read.
    pub fn set_cmd_err(&mut self, value: bool) {
        let start = 1;
        let end = 1;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
    /// `bit 2` - Set the `conf_err` field.
    ///
    /// Sensor configuration error detected (only working in normal mode). Cleared on read.
    pub fn set_conf_err(&mut self, value: bool) {
        let start = 2;
        let end = 2;
        let raw = value as _;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for ErrReg {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct RevId {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for RevId {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl RevId {
    /// `7:0` - Read the `value` field.
    ///
    /// The mask revision of the ASIC.
    #[must_use]
    pub fn value(&self) -> u8 {
        let start = 0;
        let end = 7;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `7:0` - Set the `value` field.
    ///
    /// The mask revision of the ASIC.
    pub fn set_value(&mut self, value: u8) {
        let start = 0;
        let end = 7;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for RevId {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(transparent)]
pub struct ChipId {
    #[doc(hidden)]
    /// The internal bits
    bits: [u8; 1],
}
unsafe impl ::device_driver::Fieldset for ChipId {
    const METADATA: ::device_driver::FieldsetMetadata = ::device_driver::FieldsetMetadata::new()
        .with_byte_order(::device_driver::ByteOrder::LE);
    const ZERO: Self = Self { bits: [0; 1] };
}
impl ChipId {
    /// `7:0` - Read the `value` field.
    ///
    /// The chip identification code.
    #[must_use]
    pub fn value(&self) -> u8 {
        let start = 0;
        let end = 7;
        let raw = unsafe {
            ::device_driver::ops::load::<
                u8,
                ::device_driver::ops::LE,
            >(&self.bits, start, end)
        };
        raw
    }
    /// `7:0` - Set the `value` field.
    ///
    /// The chip identification code.
    pub fn set_value(&mut self, value: u8) {
        let start = 0;
        let end = 7;
        let raw = value;
        unsafe {
            ::device_driver::ops::store::<
                u8,
                ::device_driver::ops::LE,
            >(raw, start, end, &mut self.bits)
        };
    }
}
impl Default for ChipId {
    fn default() -> Self {
        <Self as ::device_driver::Fieldset>::ZERO
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for Command {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for IirFilterCoefficient {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
    #[doc(alias = "Odr12p5")]
    Odr12P5 = 4,
    /// 25/4 Hz, 160 ms sampling period, prescaler 32.
    #[doc(alias = "Odr6p25")]
    Odr6P25 = 5,
    /// 25/8 Hz, 320 ms sampling period, prescaler 64.
    #[doc(alias = "Odr3p1")]
    Odr3P1 = 6,
    /// 25/16 Hz, 640 ms sampling period, prescaler 128.
    #[doc(alias = "Odr1p5")]
    Odr1P5 = 7,
    /// 25/32 Hz, 1.280 s sampling period, prescaler 256.
    #[doc(alias = "Odr0p78")]
    Odr0P78 = 8,
    /// 25/64 Hz, 2.560 s sampling period, prescaler 512.
    #[doc(alias = "Odr0p39")]
    Odr0P39 = 9,
    /// 25/128 Hz, 5.120 s sampling period, prescaler 1024.
    #[doc(alias = "Odr0p2")]
    Odr0P2 = 10,
    /// 25/256 Hz, 10.24 s sampling period, prescaler 2048.
    #[doc(alias = "Odr0p1")]
    Odr0P1 = 11,
    /// 25/512 Hz, 20.48 s sampling period, prescaler 4096.
    #[doc(alias = "Odr0p05")]
    Odr0P05 = 12,
    /// 25/1024 Hz, 40.96 s sampling period, prescaler 8192.
    #[doc(alias = "Odr0p02")]
    Odr0P02 = 13,
    /// 25/2048 Hz, 81.92 s sampling period, prescaler 16384.
    #[doc(alias = "Odr0p01")]
    Odr0P01 = 14,
    /// 25/4096 Hz, 163.84 s sampling period, prescaler 32768.
    #[doc(alias = "Odr0p006")]
    Odr0P006 = 15,
    /// 25/8192 Hz, 327.68 s sampling period, prescaler 65536.
    #[doc(alias = "Odr0p003")]
    Odr0P003 = 16,
    /// 25/16384 Hz, 655.36 s sampling period, prescaler 131072.
    #[doc(alias = "Odr0p0015")]
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for OdrSel {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for Oversampling {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for MeasurementMode {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
/// The timeout period for the I2C Watchdog timer.
#[doc(alias = "I2cWatchdogPeriod")]
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for I2CWatchdogPeriod {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for SpiMode {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for IntLevel {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for IntOpenDrain {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for FifoDataSelect {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for FifoMode {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
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
#[doc(hidden)]
impl ::device_driver::EnumIndex for CommandStatus {
    #[track_caller]
    fn index(&self) -> i32 {
        let index = u8::from(*self);
        index.try_into().unwrap()
    }
}
