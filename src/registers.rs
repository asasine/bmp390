//! Register definitions for the BMP390 pressure sensor.
//!
//! Values are taken from the
//! [BMP390 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf),
//! revision 1.7, from Bosch Sensortec.

use defmt::Format;

/// The BMP390 has a number of registers to provide access to the sensor's data and control various settings.
///
/// All registers have a width of 8 bits.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Register {
    /// Contains the chip identification code. The BMP390 has a value of `0x60`.
    ///
    /// # Datasheet
    /// Section 4.3.1. Register 0x00 `CHIP_ID`
    ///
    /// # Layout
    /// | Bits | Description |
    /// | ---- | ----------- |
    /// | 7:0 | Chip id. |
    CHIP_ID = 0x00,

    /// Contains the mask revision of the ASIC.
    ///
    /// This library has been tested with a BMP390 with a revision of `0x01`. If you have a different revision, please
    /// [start a discussion](https://github.com/asasine/bmp390/discussions/new/choose) on the repository so I can hear
    /// about your experience.
    ///
    /// # Datasheet
    /// Section 4.3.2. Register 0x01 `REV_ID`
    ///
    /// # Layout
    /// | Bits | Description |
    /// | ---- | ----------- |
    /// | 7:0 | Rev id. |
    REV_ID = 0x01,

    /// Contains sensor error conditions. Some bits are cleared on read. See [`ErrReg`].
    ///
    /// # Datasheet
    /// Section 4.3.3. Register 0x02 `ERR_REG`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `fatal_err` | Fatal error. |
    /// | 1 | `cmd_err` | Command execution failed. Cleared on read. |
    /// | 2 | `conf_err` | Configuration error. Only functions in normal mode. Cleared on read. |
    ERR_REG = 0x02,

    /// Contains sensor status flags. See [`Status`].
    ///
    /// # Datasheet
    /// Section 4.3.4. Register 0x03 `STATUS`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 4 | `cmd_ready` | CMD decoder status. <br/> 0: Command in progress. <br/> 1: Command decoder is ready to accept a new command. |
    /// | 5 | `drdy_press` | Data ready for pressure. <br/> It gets reset when one pressure DATA register is read out. |
    /// | 6 | `drdy_temp` | Data ready for temperature. <br/> It gets reset when one temperature DATA register is read out. |
    STATUS = 0x03,

    /// The XLSB part of the 24-bit pressure data.
    ///
    /// Pressure data is split among three registers: [`Register::DATA_0`], [`Register::DATA_1`], and
    /// [`Register::DATA_2`].
    ///
    /// The BMP390 supports burst read mode, where the data registers can be read in a single I2C transaction. In this
    /// case, a write on the first register address is followed by a read of the subsequent registers, and the BMP390
    /// automatically increments the register address until an `NMAK` byte is sent.
    ///
    /// # Datasheet
    /// Section 4.3.5. Register 0x04 .. 0x06 Pressure Data
    ///
    /// # Layout
    /// | Register | 0x06 [`Register::DATA_2`] | 0x05 [`Register::DATA_1`] | 0x04 [`Register::DATA_0`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | PRESS_MSB_23_16 | PRESS_MSB_15_8 | PRESS_MSB_7_0 |
    DATA_0 = 0x04,

    /// The LSB part of the 24-bit pressure data.
    ///
    /// Pressure data is split among three registers: [`Register::DATA_0`], [`Register::DATA_1`], and
    /// [`Register::DATA_2`].
    ///
    /// # Datasheet
    /// Section 4.3.5. Register 0x04 .. 0x06 Pressure Data
    ///
    /// # Layout
    /// | Register | 0x06 [`Register::DATA_2`] | 0x05 [`Register::DATA_1`] | 0x04 [`Register::DATA_0`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | PRESS_MSB_23_16 | PRESS_MSB_15_8 | PRESS_MSB_7_0 |
    DATA_1 = 0x05,

    /// The MSB part of the 24-bit pressure data.
    ///
    /// Pressure data is split among three registers: [`Register::DATA_0`], [`Register::DATA_1`], and
    /// [`Register::DATA_2`].
    ///
    /// # Datasheet
    /// Section 4.3.5. Register 0x04 .. 0x06 Pressure Data
    ///
    /// # Layout
    /// | Register | 0x06 [`Register::DATA_2`] | 0x05 [`Register::DATA_1`] | 0x04 [`Register::DATA_0`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | PRESS_MSB_23_16 | PRESS_MSB_15_8 | PRESS_MSB_7_0 |
    DATA_2 = 0x06,

    /// The XLSB part of the 24-bit temperature data.
    ///
    /// Temperature data is split among three registers: [`Register::DATA_3`], [`Register::DATA_4`], and
    /// [`Register::DATA_5`].
    ///
    /// # Datasheet
    /// Section 4.3.6. Register 0x07 .. 0x09 Temperature Data
    ///
    /// # Layout
    /// | Register | 0x09 [`Register::DATA_5`] | 0x08 [`Register::DATA_4`] | 0x07 [`Register::DATA_3`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | TEMP_MSB_15_8 | TEMP_MSB_7_0 | TEMP_LSB_7_0 |
    DATA_3 = 0x07,

    /// The LSB part of the 24-bit temperature data.
    ///
    /// Temperature data is split among three registers: [`Register::DATA_3`], [`Register::DATA_4`], and
    /// [`Register::DATA_5`].
    ///
    /// # Datasheet
    /// Section 4.3.6. Register 0x07 .. 0x09 Temperature Data
    ///
    /// # Layout
    /// | Register | 0x09 [`Register::DATA_5`] | 0x08 [`Register::DATA_4`] | 0x07 [`Register::DATA_3`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | TEMP_MSB_15_8 | TEMP_MSB_7_0 | TEMP_LSB_7_0 |
    DATA_4 = 0x08,

    /// The MSB part of the 24-bit temperature data.
    ///
    /// Temperature data is split among three registers: [`Register::DATA_3`], [`Register::DATA_4`], and
    /// [`Register::DATA_5`].
    ///
    /// # Datasheet
    /// Section 4.3.6. Register 0x07 .. 0x09 Temperature Data
    ///
    /// # Layout
    /// | Register | 0x09 [`Register::DATA_5`] | 0x08 [`Register::DATA_4`] | 0x07 [`Register::DATA_3`] |
    /// | -------- | ----------- | ----------- | ----------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | TEMP_MSB_15_8 | TEMP_MSB_7_0 | TEMP_LSB_7_0 |
    DATA_5 = 0x09,

    /// The least significant bits of the 24-bit sensor time data.
    ///
    /// Sensor time data is split among three registers: [`Register::SENSORTIME_0`], [`Register::SENSORTIME_1`], and
    /// [`Register::SENSORTIME_2`].
    ///
    /// # Datasheet
    /// Section 4.3.7. Register 0x0C .. 0x0E Sensor Time Data
    ///
    /// # Layout
    /// | Register | 0x0E [`Register::SENSORTIME_2`] | 0x0D [`Register::SENSORTIME_1`] | 0x0C [`Register::SENSORTIME_0`] |
    /// | -------- | ----------------- | ----------------- | ----------------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | sensor_time_23_16 | sensor_time_15_8 | sensor_time_7_0 |
    SENSORTIME_0 = 0x0C,

    /// The middle 8 bits of the 24-bit sensor time data.
    ///
    /// Sensor time data is split among three registers: [`Register::SENSORTIME_0`], [`Register::SENSORTIME_1`], and
    /// [`Register::SENSORTIME_2`].
    ///
    /// # Datasheet
    /// Section 4.3.7. Register 0x0C .. 0x0E Sensor Time Data
    ///
    /// # Layout
    /// | Register | 0x0E [`Register::SENSORTIME_2`] | 0x0D [`Register::SENSORTIME_1`] | 0x0C [`Register::SENSORTIME_0`] |
    /// | -------- | ----------------- | ----------------- | ----------------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | sensor_time_23_16 | sensor_time_15_8 | sensor_time_7_0 |
    SENSORTIME_1 = 0x0D,

    /// The most significant bits of the 24-bit sensor time data.
    ///
    /// Sensor time data is split among three registers: [`Register::SENSORTIME_0`], [`Register::SENSORTIME_1`], and
    /// [`Register::SENSORTIME_2`].
    ///
    /// # Datasheet
    /// Section 4.3.7. Register 0x0C .. 0x0E Sensor Time Data
    ///
    /// # Layout
    /// | Register | 0x0E [`Register::SENSORTIME_2`] | 0x0D [`Register::SENSORTIME_1`] | 0x0C [`Register::SENSORTIME_0`] |
    /// | -------- | ----------------- | ----------------- | ----------------- |
    /// | Bit | 7..0 | 7..0 | 7..0 |
    /// | Name | sensor_time_23_16 | sensor_time_15_8 | sensor_time_7_0 |
    SENSORTIME_2 = 0x0E,

    /// Contains sensor status flags for certain events. Cleared on read. See [`Event`].
    ///
    /// # Datasheet
    /// Section 4.3.8. Register 0x10 `EVENT`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `por_detected` | `1` after device power up or soft reset. Cleaed on read. |
    /// | 1 | `itf_act_pt` | `1` when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on read. |
    EVENT = 0x10,

    /// Contains interrupt statuses. Cleared on read. See [`IntStatus`].
    ///
    /// # Datasheet
    /// Section 4.3.9. Register 0x11 `INT_STATUS`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `fwm_int` | FIFO watermark interrupt. |
    /// | 1 | `ffull_int` | FIFO full interrupt. |
    /// | 3 | `drdy` | Data ready interrupt. |
    INT_STATUS = 0x11,

    /// Contains the LSB for the current fill level of the FIFO buffer. Its size is 9 bits for 512 bytes and is
    /// therefore split in two registers: [`Register::FIFO_LENGTH_0`] and [`Register::FIFO_LENGTH_1`].
    ///
    /// # Datasheet
    /// Section 4.3.10. Register 0x12 .. 0x13 `FIFO_LENGTH`
    ///
    /// # Layout
    /// | Register | 0x13 [`Register::FIFO_LENGTH_1`] | 0x12 [`Register::FIFO_LENGTH_0`] |
    /// | -------- | ------------------- | ------------------- |
    /// | Bit | 0 | 7..0 |
    /// | Name | fifo_byte_counter_11_8 | fifo_byte_counter_7_0 |
    FIFO_LENGTH_0 = 0x12,

    /// Contains the most significant bit for the current fill level of the FIFO buffer. Its size is 9 bits for 512
    /// bytes and is therefore split in two registers: [`Register::FIFO_LENGTH_0`] and [`Register::FIFO_LENGTH_1`].
    ///
    /// # Datasheet
    /// Section 4.3.10. Register 0x12 .. 0x13 `FIFO_LENGTH`
    ///
    /// # Layout
    /// | Register | 0x13 [`Register::FIFO_LENGTH_1`] | 0x12 [`Register::FIFO_LENGTH_0`] |
    /// | -------- | ------------------- | ------------------- |
    /// | Bit | 0 | 7..0 |
    /// | Name | fifo_byte_counter_8 | fifo_byte_counter_7_0 |
    FIFO_LENGTH_1 = 0x13,

    /// Contains the start of the FIFO data range.
    ///
    /// The FIFO can store up to 512 bytes of data. The data is stored in the order of the measurements, with the
    /// oldest data at the start of the FIFO. The BMP390 supports burst read mode, where the data registers can be read
    /// in a single I2C transaction. In this case, a write on this register address is followed by a read equal to the
    /// number of bytes in the FIFO, as reported by [`Register::FIFO_LENGTH_0`] and [`Register::FIFO_LENGTH_1`]. The
    /// BMP390 automatically increments the register address until an `NMAK` byte is sent.
    ///
    /// # Datasheet
    /// Section 4.3.11. Register 0x14 `FIFO_DATA`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 7:0 | `fifo_data` | FIFO read data. |
    FIFO_DATA = 0x14,

    /// Contains the LSB for the FIFO watermark level. The watermark size is 9 bits and therefore written across two
    /// registers: [`Register::FIFO_WTM_0`] and [`Register::FIFO_WTM_1`].
    ///
    /// # Datasheet
    /// Section 4.3.12. Register 0x15 .. 0x16 `FIFO_WTM`
    ///
    /// # Layout
    /// | Register | 0x16 [`Register::FIFO_WTM_1`] | 0x15 [`Register::FIFO_WTM_0`] |
    /// | -------- | ---------------- | ---------------- |
    /// | Bit | 0 | 7..0 |
    /// | Name | fifo_wtm_8 | fifo_wtm_7_0 |
    FIFO_WTM_0 = 0x15,

    /// Contains the most significant bit for the FIFO watermark level. The watermark size is 9 bits and therefore
    /// written across two registers: [`Register::FIFO_WTM_0`] and [`Register::FIFO_WTM_1`].
    ///
    /// # Datasheet
    /// Section 4.3.12. Register 0x15 .. 0x16 `FIFO_WTM`
    ///
    /// # Layout
    /// | Register | 0x16 [`Register::FIFO_WTM_1`] | 0x15 [`Register::FIFO_WTM_0`] |
    /// | -------- | ---------------- | ---------------- |
    /// | Bit | 0 | 7..0 |
    /// | Name | fifo_wtm_8 | fifo_wtm_7_0 |
    FIFO_WTM_1 = 0x16,

    /// Contains the FIFO frame content configuration.
    ///
    /// # Datasheet
    /// Section 4.3.13. Register 0x17 `FIFO_CONFIG_1`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `fifo_mode` | Enables or disables the FIFO. <br/> 0: disable. <br/> 1: enable FIFO mode. |
    /// | 1 | `fifo_stop_on_full` | Stops writing samples to the FIFO when it's full. <br/> 0: FIFO continues to write when full. <br/> 1: stop writing when full. |
    /// | 2 | `fifo_time_en` | Return sensortime frame after the last valid data frame. <br/> 0: do not return sensortime frame. <br/> 1: return sensortime frame. |
    /// | 3 | `fifo_press_en` | Store pressure data in FIFO. <br/> 0: no pressure data is stored. <br/> 1: store pressure data. |
    /// | 4 | `fifo_temp_en` | Store temperature data in FIFO. <br/> 0: no temperature data is stored. <br/> 1: store temperature data. |
    FIFO_CONFIG_1 = 0x17,

    /// Contains the FIFO frame content configuration.
    ///
    /// # Datasheet
    /// Section 4.3.14. Register 0x18 `FIFO_CONFIG_2`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 2..0 | `fifo_subsampling` | FIFO downsampling selection for pressure and temperature data. Factor is `2 ^ fifo_subsampling` |
    /// | 4..3 | `data_select` | Select data source for pressure and temperature. <br/> `0`: unfiltered data (compensated or uncompensated) <br/> `23`: filtered data (compensated or uncompensated) <br/> `11` / `10`: reserved, same as for `unfilt` |
    FIFO_CONFIG_2 = 0x18,

    /// Interrupt configuration, affecting [`Register::INT_STATUS`] and the `INT` pin.
    ///
    /// # Datasheet
    /// Section 4.3.15. Register 0x19 `INT_CTRL`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `int_od` | Configure output: open-drain (`1`) or push-pull (`0`). |
    /// | 1 | `int_level` | Level of `INT` pin: active high (`1`) or active low (`0`). |
    /// | 2 | `int_latch` | Latching of `INT` pin and [`Register::INT_STATUS`]: enabled (`1`) or disabled (`0`). |
    /// | 3 | `fwtm_en` | Enable FIFO watermark interrupt: enabled (`1`) or disabled (`0`). |
    /// | 4 | `ffull_en` | Enable FIFO full interrupt: enabled (`1`) or disabled (`0`). |
    /// | 5 | `int_ds` | low (`0`) or high (`1`) |
    /// | 6 | `drdy_en` | Enable temperature and pressure data ready interrupt: enabled (`1`) or disabled (`0`). |
    INT_CTRL = 0x19,

    /// Contains serial interface settings.
    ///
    /// # Datasheet
    /// Section 4.3.16. Register 0x1A `IF_CONF`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `spi3` | Configure SPI Interface Mode for the primary interface. <br/> 0: SPI 4-wire mode. <br/> 1: SPI 3-wire mode. |
    /// | 1 | `i2c_wdt_en` | Enable I2C watchdog timer, backed by NVM: enabled (`1`) or disabled (`0`). |
    /// | 2 | `i2c_wdt_sel` | Select timer period for I2C watchdog timer: <br/> 0: short (1.25ms) <br/> 1: long (40ms) |
    IF_CONF = 0x1A,

    /// Enables or disables pressure and temperature measurements and sets the power mode. See [`PowerControl`].
    ///
    /// # Datasheet
    /// Section 4.3.17. Register 0x1B `PWR_CTRL`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 0 | `press_en` | Enable pressure sensor: enabled (`1`) or disabled (`0`). |
    /// | 1 | `temp_en` | Enable temperature sensor: enabled (`1`) or disabled (`0`). |
    /// | 5:4 | `mode` | See [`PowerMode`] |
    PWR_CTRL = 0x1B,

    /// Controls oversampling settings for pressure and temperature measurements. See [`Osr`].
    ///
    /// # Datasheet
    /// Section 4.3.18. Register 0x1C `OSR`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 2..0 | `osr_press` | Oversampling setting for pressure measurement. See [`Oversampling`]. |
    /// | 5..3 | `osr_temp` | Oversampling setting for temperature measurement. See [`Oversampling`]. |
    OSR = 0x1C,

    /// Configures the output data rates by means of setting the subdivision/subsampling. See [`Odr`].
    ///
    /// # Datasheet
    /// Section 4.3.19. Register 0x1D `ODR`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 4..0 | `odr_sel` | See [`Odr::odr_sel`] |
    ODR = 0x1D,

    /// Controls the IIR filter coefficients.
    ///
    /// # Datasheet
    /// Section 4.3.21. Register 0x1F `CONFIG`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 3..1 | `iir_filter` | Filter coefficient for IIR filter. See [`IirFilter`]. |
    CONFIG = 0x1F,

    /// [`Register::NVM_PAR_T1_0`] through [`Register::NVM_PAR_P11`] contain calibration data for the sensor.
    ///
    /// # Datasheet
    /// Section 4.3.22. Register 0x30 .. 0x57 calibration data
    NVM_PAR_T1_0 = 0x31,
    NVM_PAR_T1_1 = 0x32,
    NVM_PAR_T2_0 = 0x33,
    NVM_PAR_T2_1 = 0x34,
    NVM_PAR_T3 = 0x35,
    NVM_PAR_P1_0 = 0x36,
    NVM_PAR_P1_1 = 0x37,
    NVM_PAR_P2_0 = 0x38,
    NVM_PAR_P2_1 = 0x39,
    NVM_PAR_P3 = 0x3A,
    NVM_PAR_P4 = 0x3B,
    NVM_PAR_P5_0 = 0x3C,
    NVM_PAR_P5_1 = 0x3D,
    NVM_PAR_P6_0 = 0x3E,
    NVM_PAR_P6_1 = 0x3F,
    NVM_PAR_P7 = 0x40,
    NVM_PAR_P8 = 0x41,
    NVM_PAR_P9_0 = 0x42,
    NVM_PAR_P9_1 = 0x43,
    NVM_PAR_P10 = 0x44,
    NVM_PAR_P11 = 0x45,

    /// A writable register to issue commands to the sensor. See [`Command`].
    ///
    /// # Datasheet
    /// Section 4.3.23. Register 0x7E `CMD`
    ///
    /// # Layout
    /// | Bit | Name | Description |
    /// | --- | ---- | ----------- |
    /// | 7..0 | `cmd` | Command to be executed. See [`Command`]. |
    CMD = 0x7E,
}

impl From<Register> for u8 {
    /// Convert a [`Register`] into its memory address for writing to the I2C bus.
    fn from(register: Register) -> u8 {
        register as u8
    }
}

/// [`Register::ERR_REG`] contains sensor error conditions.
///
/// # Datasheet
/// Section 4.3.3. Register 0x02 `ERR_REG`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 0 | `fatal_err` | [`ErrReg::fatal_err`] |
/// | 1 | `cmd_err` | [`ErrReg::cmd_err`] |
/// | 2 | `conf_err` | [`ErrReg::conf_err`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct ErrReg {
    /// Indicates if there is a fatal error.
    pub fatal_err: bool,

    /// Indicates if command execution failed. Cleared on read.
    pub cmd_err: bool,

    /// Indicates if there is a configuration error. This only functions in [`PowerMode::Normal`]. Cleared on read.
    pub conf_err: bool,
}

impl From<u8> for ErrReg {
    /// Convert a [`u8`] from [`Register::ERR_REG`] to its matching [`ErrReg`].
    fn from(value: u8) -> ErrReg {
        ErrReg {
            fatal_err: (value & 0b01) != 0,
            cmd_err: (value & 0b10) != 0,
            conf_err: (value & 0b100) != 0,
        }
    }
}

impl From<ErrReg> for u8 {
    /// Convert this type to a value that mirrors [`Register::ERR_REG`].
    fn from(err_reg: ErrReg) -> u8 {
        let mut value = 0;
        if err_reg.fatal_err {
            value |= 1 << 0;
        }

        if err_reg.cmd_err {
            value |= 1 << 1;
        }

        if err_reg.conf_err {
            value |= 1 << 2;
        }

        value
    }
}

/// [`Register::STATUS`] contains sensor status flags.
///
/// # Datasheet
/// Section 4.3.4. Register 0x03 `STATUS`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 4 | `cmd_ready` | [`Status::cmd_ready`] |
/// | 5 | `drdy_press` | [`Status::drdy_press`] |
/// | 6 | `drdy_temp` | [`Status::drdy_temp`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Status {
    /// The command decoder status. Indicates if a [`Command`] is in progress (`false`) or if [`Register::CMD`] is
    /// ready to accept a new command (`true`).
    pub cmd_ready: bool,

    /// Indicates if there is new pressure data available. It gets reset when any of the pressure `DATA_*` registers
    /// are read out.
    pub drdy_press: bool,

    /// Indicates if there is new temperature data available. It gets reset when any of the temperature `DATA_*`
    /// registers are read out.
    pub drdy_temp: bool,
}

impl From<u8> for Status {
    /// Convert a [`u8`] from [`Register::STATUS`] to its matching [`Status`].
    fn from(value: u8) -> Status {
        Status {
            cmd_ready: (value & (1 << 4)) != 0,
            drdy_press: (value & (1 << 5)) != 0,
            drdy_temp: (value & (1 << 6)) != 0,
        }
    }
}

/// [`Register::EVENT`] contains status flags for certain events. These flags are cleared on read.
///
/// # Datasheet
/// Section 4.3.5. Register 0x10 `EVENT`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 0 | `por_detected` | [`Event::por_detected`] |
/// | 1 | `itf_act_pt` | [`Event::itf_act_pt`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Event {
    /// Indicates if a power-on reset was detected. Cleared on read.
    pub por_detected: bool,

    /// Indicates when a serial interface transaction occurs during a pressure or temperature conversion. Cleared on
    /// read.
    pub itf_act_pt: bool,
}

impl From<u8> for Event {
    /// Convert a [`u8`] from [`Register::EVENT`] to its matching [`Event`].
    fn from(value: u8) -> Event {
        Event {
            por_detected: (value & 0b01) != 0,
            itf_act_pt: (value & 0b10) != 0,
        }
    }
}

impl From<Event> for u8 {
    /// Convert this type to a value that mirrors [`Register::EVENT`].
    fn from(event: Event) -> u8 {
        let mut value = 0;
        if event.por_detected {
            value |= 1 << 0;
        }

        if event.itf_act_pt {
            value |= 1 << 1;
        }

        value
    }
}

/// [`Register::INT_STATUS`] shows interrupt statuses and is cleared on read.
///
/// # Datasheet
/// Section 4.3.6. Register 0x11 `INT_STATUS`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 0 | `fwm_int` | [`IntStatus::fwm_int`] |
/// | 1 | `ffull_int` | [`IntStatus::ffull_int`] |
/// | 3 | `drdy` | [`IntStatus::drdy`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct IntStatus {
    /// Indicates if the FIFO watermark interrupt is active. Cleared on read.
    pub fwm_int: bool,

    /// Indicates if the FIFO full interrupt is active. Cleared on read.
    pub ffull_int: bool,

    /// Indicates if the data ready interrupt is active. Cleared on read.
    pub drdy: bool,
}

impl From<u8> for IntStatus {
    /// Convert a [`u8`] from [`Register::INT_STATUS`] to its matching [`IntStatus`].
    fn from(value: u8) -> IntStatus {
        IntStatus {
            fwm_int: (value & 0b01) != 0,
            ffull_int: (value & 0b10) != 0,
            drdy: (value & 0b1000) != 0,
        }
    }
}

impl From<IntStatus> for u8 {
    /// Convert this type to a value that mirrors [`Register::INT_STATUS`].
    fn from(int_status: IntStatus) -> u8 {
        let mut value = 0;
        if int_status.fwm_int {
            value |= 1 << 0;
        }

        if int_status.ffull_int {
            value |= 1 << 1;
        }

        if int_status.drdy {
            value |= 1 << 3;
        }

        value
    }
}

/// The interrupt output type for the `INT` pin.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum IntOd {
    /// Open-drain output.
    OpenDrain = 1,

    /// Push-pull output.
    PushPull = 0,
}

/// The interrupt level for the `INT` pin.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum IntLevel {
    /// Active high.
    High = 1,

    /// Active low.
    Low = 0,
}

/// [`Register::INT_CTRL`] configures the interrupt settings and affects [`Register::INT_STATUS`] and the `INT` pin.
///
/// # Datasheet
/// Section 4.3.15. Register 0x19 `INT_CTRL`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 0 | `int_od` | [`IntControl::output`] |
/// | 1 | `int_level` | [`IntControl::level`] |
/// | 2 | `int_latch` | [`IntControl::latch`] |
/// | 3 | `fwtm_en` | [`IntControl::fifo_watermark`] |
/// | 4 | `ffull_en` | [`IntControl::fifo_full`] |
/// | 5 | `int_ds` | [`IntControl::int_ds`] |
/// | 6 | `drdy_en` | [`IntControl::drdy_en`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct IntControl {
    /// Configure output: open-drain or push-pull.
    pub output: IntOd,

    /// Level of `INT` pin: active high or active low.
    pub level: IntLevel,

    /// Latching of `INT` pin and [`Register::INT_STATUS`]: enabled (`true`) or disabled (`false`).
    pub latch: bool,

    /// Enable FIFO watermark interrupt: enabled (`true`) or disabled (`false`).
    ///
    /// This interrupt is triggered when the FIFO fill level is equal to or higher than the watermark level.
    /// The watermark level is set by [`Register::FIFO_WTM_0`] and [`Register::FIFO_WTM_1`].
    pub fifo_watermark: bool,

    /// Enable FIFO full interrupt: enabled (`true`) or disabled (`false`).
    ///
    /// This interrupt is triggered when the FIFO is full.
    pub fifo_full: bool,

    /// Low (`false`) or high (`true`) interrupt level.
    pub int_ds: bool,

    /// Enable temperature and pressure data ready interrupt: enabled (`true`) or disabled (`false`).
    pub drdy_en: bool,
}

impl From<IntControl> for u8 {
    /// Convert this type to a value that mirrors [`Register::INT_CTRL`].
    fn from(control: IntControl) -> u8 {
        let mut value = 0;
        if control.output == IntOd::OpenDrain {
            value |= 1 << 0;
        }

        if control.level == IntLevel::High {
            value |= 1 << 1;
        }

        if control.latch {
            value |= 1 << 2;
        }

        if control.fifo_watermark {
            value |= 1 << 3;
        }

        if control.fifo_full {
            value |= 1 << 4;
        }

        if control.int_ds {
            value |= 1 << 5;
        }

        if control.drdy_en {
            value |= 1 << 6;
        }

        value
    }
}

/// The BMP390 offers three power modes: sleep mode, forced mode and normal mode.
///
/// These are used to set the [`PowerControl::mode`] bits in the [`Register::PWR_CTRL`] register.
///
/// # Datasheet
/// 1. Section 3.3. Power modes
/// 1. Section 4.3.17. Register 0x1B `PWR_CTRL`, Table 42.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PowerMode {
    /// Sleep mode is set by default after power on reset. In sleep mode, no measurements are performed and power
    /// consumption (`I_DDSL`) is at a minimum. All registers are accessible; Chip-ID and compensation coefficients can
    /// be read.
    Sleep = 0b00,

    /// In forced mode, a single measurement is performed according to selected measurement and filter options. When
    /// the measurement is finished, the sensor returns to sleep mode and the measurement results can be obtained from
    /// the data registers. For a next measurement, forced mode needs to be selected again. Forced mode is recommended
    /// for applications which require low sampling rate or host-based synchronization.
    Forced = 0b01,

    /// Normal mode continuously cycles between an (active) measurement period and an (inactive) standby period. The
    /// measurement rate is set in the [`Register::ODR`] register (see 4.3.19), where various prescaler for sample
    /// frequencies can be selected (see 3.3.3).
    ///
    /// After setting the mode, measurement and filter options, the last measurement results can be obtained from the
    /// [`Bmp390::measure`](`crate::Bmp390::measure`) without the need of further write accesses. Normal mode is recommended when using the IIR
    /// filter, and useful for applications in which short-term disturbances (e.g. blowing into the sensor) should be
    /// filtered.
    Normal = 0b11,
}

impl From<PowerMode> for u8 {
    /// Convert this type to a value that can be written to the [`Register::PWR_CTRL`] register.
    /// This is the same as the enum value, and must be bit shifted into the correct position.
    fn from(mode: PowerMode) -> u8 {
        mode as u8
    }
}

impl From<u8> for PowerMode {
    /// Convert a [`u8`] into a [`PowerMode`]. Only the lower two bits are used.
    fn from(value: u8) -> PowerMode {
        match value & 0b11 {
            0b00 => PowerMode::Sleep,
            0b01 | 0b10 => PowerMode::Forced,
            0b11 => PowerMode::Normal,
            _ => unreachable!(),
        }
    }
}

/// [`Register::PWR_CTRL`] enables or disables pressure and temperature measurements and sets the [`PowerMode`].
///
/// # Datasheet
/// Section 4.3.17. Register 0x1B `PWR_CTRL`
///
/// # Layout
/// | Bit | Name | Field |
/// | --- | ---- | ----- |
/// | 0 | `press_en` | [`PowerControl::enable_pressure`] |
/// | 1 | `temp_en` | [`PowerControl::enable_temperature`] |
/// | 5:4 | `mode` | [`PowerControl::mode`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct PowerControl {
    /// Enable the pressure sensor.
    pub enable_pressure: bool,

    /// Enable the temperature sensor.
    pub enable_temperature: bool,

    /// The power mode to set.
    pub mode: PowerMode,
}

impl From<PowerControl> for u8 {
    /// Convert this type to a value that can be written to [`Register::PWR_CTRL`].
    fn from(control: PowerControl) -> u8 {
        let mut value = (control.mode as u8) << 4;
        if control.enable_pressure {
            value |= 1 << 0;
        }

        if control.enable_temperature {
            value |= 1 << 1;
        }

        value
    }
}

impl From<u8> for PowerControl {
    /// Convert a [`u8`] from [`Register::PWR_CTRL`] to its matching [`PowerControl`].
    fn from(value: u8) -> PowerControl {
        let enable_pressure = (value & 0b01) != 0; // bit 0
        let enable_temperature = (value & 0b10) != 0; // bit 1
        let mode = (value >> 4).into(); // bits 5:4

        PowerControl {
            mode,
            enable_pressure,
            enable_temperature,
        }
    }
}

/// Oversampling settings for pressure and temperature measurements.
///
/// # Datasheet
/// 1. Section 3.3.4. Oversampling
/// 1. Section 4.3.18. Register 0x1C `OSR`, Table 43.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum Oversampling {
    /// No oversampling.
    None = 0b000,

    /// 2x oversampling.
    X2 = 0b001,

    /// 4x oversampling.
    X4 = 0b010,

    /// 8x oversampling.
    X8 = 0b011,

    /// 16x oversampling.
    X16 = 0b100,

    /// 32x oversampling.
    X32 = 0b101,
}

impl Oversampling {
    /// No oversampling. An alias of [`Oversampling::None`].
    pub const X1: Oversampling = Oversampling::None;
}

impl From<Oversampling> for u8 {
    /// Convert this type to a value that can be written to one portion of [`Register::OSR`].
    fn from(oversampling: Oversampling) -> u8 {
        oversampling as u8
    }
}

impl TryFrom<u8> for Oversampling {
    type Error = u8;

    /// Try to convert a [`u8`] from [`Register::OSR`] to its matching [`Oversampling`]. Only the lower three bits are
    /// used. If the value is not recognized, the masked bits are returned as an error.
    fn try_from(value: u8) -> Result<Oversampling, Self::Error> {
        match value & 0b111 {
            0b000 => Ok(Oversampling::None),
            0b001 => Ok(Oversampling::X2),
            0b010 => Ok(Oversampling::X4),
            0b011 => Ok(Oversampling::X8),
            0b100 => Ok(Oversampling::X16),
            0b101 => Ok(Oversampling::X32),
            value => Err(value),
        }
    }
}

/// [`Register::OSR`] controls the oversampling settings for pressure and temperature measurements.
///
/// Noise depends on the [`Oversampling`] and filter settings selected. See Section 3.4.4.
///
/// # Datasheet
/// Section 4.3.18. Register 0x1C `OSR`
///
/// # Layout
/// | Bits | Name | Field |
/// | ---- | ---- | ----- |
/// | 2:0 | `osr_p` | [`Osr::pressure`] |
/// | 5:3 | `osr_t` | [`Osr::temperature`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Osr {
    /// Oversampling setting for pressure measurements.
    pub pressure: Oversampling,

    /// Oversampling setting for temperature measurements.
    pub temperature: Oversampling,
}

impl From<Osr> for u8 {
    /// Convert this type to a value that can be written to [`Register::OSR`].
    fn from(osr: Osr) -> u8 {
        (osr.pressure as u8) | ((osr.temperature as u8) << 3)
    }
}

/// [`Register::ODR`] sets the configuration of the output data rates by means of setting the subdivision/subsampling.
///
/// # Datasheet
/// Section 4.3.19. Register 0x1D `ODR`
///
/// # Layout
/// | Bits | Name | Field |
/// | ---- | ---- | ----- |
/// | 4:0 | `odr_sel` | [`Odr::odr_sel`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Odr {
    /// The subdivision factor for pressure and temperature measurements is `2^odr_sel`. Allowed values are 0..=17.
    /// Other values are saturated at 17.
    pub odr_sel: OdrSel,
}

impl From<Odr> for u8 {
    /// Convert this type to a value that can be written to [`Register::ODR`].
    fn from(odr: Odr) -> u8 {
        odr.odr_sel as u8
    }
}

/// The output data rate (ODR) is the rate at which the sensor provides new data.
///
/// # Datasheet
/// Section 4.3.20. Control settings for odr_sel, Table 45.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[allow(non_camel_case_types)]
pub enum OdrSel {
    /// 200 Hz, sampling period 5 ms, prescaler 1.
    ODR_200 = 0x00,

    /// 100 Hz, sampling period 10 ms, prescaler 2.
    ODR_100 = 0x01,

    /// 50 Hz, sampling period 20 ms, prescaler 4.
    ODR_50 = 0x02,

    /// 25 Hz, sampling period 40 ms, prescaler 8.
    ODR_25 = 0x03,

    /// 12.5 Hz (25/2), sampling period 80 ms, prescaler 16.
    ODR_12p5 = 0x04,

    /// 6.25 Hz (25/4), sampling period 160 ms, prescaler 32.
    ODR_6p25 = 0x05,

    /// 3.1 Hz (25/8), sampling period 320 ms, prescaler 64.
    ODR_3p1 = 0x06,

    // 1.5 Hz (25/16), sampling period 640 ms, prescaler 128.
    ODR_1p5 = 0x07,

    /// 0.78 Hz (25/32), sampling period 1.280 s, prescaler 256.
    ODR_0p78 = 0x08,

    /// 0.39 Hz (25/64), sampling period 2.560 s, prescaler 512.
    ODR_0p39 = 0x09,

    /// 0.2 Hz (25/128), sampling period 5.120 s, prescaler 1024.
    ODR_0p2 = 0x0A,

    /// 0.1 Hz (25/256), sampling period 10.24 s, prescaler 2048.
    ODR_0p1 = 0x0B,

    /// 0.05 Hz (25/512), sampling period 20.48 s, prescaler 4096.
    ODR_0p05 = 0x0C,

    /// 0.02 Hz (25/1024), sampling period 40.96 s, prescaler 8192.
    ODR_0p02 = 0x0D,

    /// 0.01 Hz (25/2048), sampling period 81.92 s, prescaler 16384.
    ODR_0p01 = 0x0E,

    /// 0.006 Hz (25/4096), sampling period 163.84 s, prescaler 32768.
    ODR_0p006 = 0x0F,

    /// 0.003 Hz (25/8192), sampling period 327.68 s, prescaler 65536.
    ODR_0p003 = 0x10,

    /// 0.0015 Hz (25/16384), sampling period 655.36 s, prescaler 131072.
    ODR_0p0015 = 0x11,
}

impl From<OdrSel> for u8 {
    /// Convert this type to a value that can be written to [`Register::ODR`].
    fn from(odr: OdrSel) -> u8 {
        odr as u8
    }
}

#[cfg(feature = "embassy-time")]
impl From<OdrSel> for embassy_time::Duration {
    /// Convert this type to an [`embassy_time::Duration`] that represents the sampling period.
    fn from(odr: OdrSel) -> embassy_time::Duration {
        match odr {
            OdrSel::ODR_200 => embassy_time::Duration::from_millis(5),
            OdrSel::ODR_100 => embassy_time::Duration::from_millis(10),
            OdrSel::ODR_50 => embassy_time::Duration::from_millis(20),
            OdrSel::ODR_25 => embassy_time::Duration::from_millis(40),
            OdrSel::ODR_12p5 => embassy_time::Duration::from_millis(80),
            OdrSel::ODR_6p25 => embassy_time::Duration::from_millis(160),
            OdrSel::ODR_3p1 => embassy_time::Duration::from_millis(320),
            OdrSel::ODR_1p5 => embassy_time::Duration::from_millis(640),
            OdrSel::ODR_0p78 => embassy_time::Duration::from_millis(1_280),
            OdrSel::ODR_0p39 => embassy_time::Duration::from_millis(2_560),
            OdrSel::ODR_0p2 => embassy_time::Duration::from_millis(5_120),
            OdrSel::ODR_0p1 => embassy_time::Duration::from_millis(10_240),
            OdrSel::ODR_0p05 => embassy_time::Duration::from_millis(20_480),
            OdrSel::ODR_0p02 => embassy_time::Duration::from_millis(40_960),
            OdrSel::ODR_0p01 => embassy_time::Duration::from_millis(81_920),
            OdrSel::ODR_0p006 => embassy_time::Duration::from_millis(163_840),
            OdrSel::ODR_0p003 => embassy_time::Duration::from_millis(327_680),
            OdrSel::ODR_0p0015 => embassy_time::Duration::from_millis(655_360),
        }
    }
}

/// The IIR filter can suppress short-term pressure disturbances by filtering internally.
///
/// The environmental pressure is subject to many short-term changes, caused e.g. by slamming of a door or window, or
/// wind blowing into the sensor. To suppress these disturbances in the output data without causing additional
/// interface traffic and processor work load, the BMP390 features an internal IIR filter. It effectively reduces the
/// bandwidth of the output signals.
///
/// The IIR filter can be configured using [`Config::iir_filter`].
///
/// # Datasheet
/// 1. Section 3.4.3. IIR filter
/// 1. Section 4.3.21. Register 0x1F `CONFIG`, Table 46.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
#[allow(non_camel_case_types)]
pub enum IirFilter {
    /// Filter coefficient is 0: bypass mode.
    coef_0 = 0b000,

    /// Filter coefficient is 1.
    coef_1 = 0b001,

    /// Filter coefficient is 3.
    coef_3 = 0b010,

    /// Filter coefficient is 7.
    coef_7 = 0b011,

    /// Filter coefficient is 15.
    coef_15 = 0b100,

    /// Filter coefficient is 31.
    coef_31 = 0b101,

    /// Filter coefficient is 63.
    coef_63 = 0b110,

    /// Filter coefficient is 127.
    coef_127 = 0b111,
}

impl IirFilter {
    /// The filter coefficient for bypass mode. This is an alias of [`IirFilter::coef_0`].
    pub const BYPASS: IirFilter = IirFilter::coef_0;
}

impl From<IirFilter> for u8 {
    /// Convert this type to a value that can be written to [`Register::CONFIG`].
    fn from(filter: IirFilter) -> u8 {
        filter as u8
    }
}

/// [`Register::CONFIG`] controls the IIR filter coefficients.
///
/// # Datasheet
/// Section 4.3.21. Register 0x1F `CONFIG`
///
/// # Layout
/// | Bits | Name | Field |
/// | ---- | ---- | ----- |
/// | 3:1 | `iir_filter` | [`Config::iir_filter`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Config {
    /// Filter coefficient for the IIR filter.
    pub iir_filter: IirFilter,
}

impl From<Config> for u8 {
    /// Convert this type to a value that can be written to [`Register::CONFIG`].
    fn from(config: Config) -> u8 {
        (config.iir_filter as u8) << 1
    }
}

/// Available commands that can be written to [`Register::CMD`].
///
/// # Datasheet
/// Section 4.3.22. Register 0x7E `CMD`, Table 48.
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum Command {
    /// Reserved. No command.
    Nop = 0x00,

    /// Clears all data in the FIFO, does not change [`Register::FIFO_CONFIG_1`] or [`Register::FIFO_CONFIG_2`].
    FifoFlush = 0xB0,

    /// Triggers a soft reset of the device. All user configuration settings are overwritten with their default state.
    /// FIFO data is cleared.
    SoftReset = 0xB6,
}

/// [`Register::CMD`] issues commands to the BMP390.
///
/// # Datasheet
/// Section 4.3.22. Register 0x7E `CMD`
///
/// # Layout
/// | Bits | Name | Field |
/// | ---- | ---- | ----- |
/// | 7:0 | `cmd` | [`Cmd::command`] |
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub struct Cmd {
    /// The command to execute.
    pub command: Command,
}

impl From<Cmd> for u8 {
    /// Convert this type to a value that can be written to [`Register::CMD`].
    fn from(cmd: Cmd) -> u8 {
        cmd.command as u8
    }
}
