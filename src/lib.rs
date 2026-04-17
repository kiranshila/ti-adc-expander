#![no_std]

use device_driver::AsyncRegisterInterface;
use embedded_hal_async::{i2c::I2c, spi::SpiDevice};

// ── Chip typestates ───────────────────────────────────────────────────────────

mod sealed {
    pub trait Chip {}
}

/// Marker trait implemented by all supported chips in this family.
pub trait Chip: sealed::Chip {}

/// TLA2518 (SPI) / TLA2528 (I²C): base feature set.
/// Supports ADC conversion, GPIO, oversampling, and sequencing.
/// No alert output, no statistics registers, no DWC.
pub struct Tla252x;

/// ADS7038 (SPI) / ADS7138 (I²C): adds digital window comparator, per-channel
/// alert thresholds, statistics (min/max/recent) registers, and GPO trigger.
pub struct Ads7x38;

/// ADS7028 (SPI) / ADS7128 (I²C): full feature set — adds RMS computation and
/// zero-crossing detection on top of all ADS7x38 capabilities.
pub struct Ads7x28;

impl sealed::Chip for Tla252x {}
impl sealed::Chip for Ads7x38 {}
impl sealed::Chip for Ads7x28 {}
impl Chip for Tla252x {}
impl Chip for Ads7x38 {}
impl Chip for Ads7x28 {}

/// Implemented by chips with alert output, digital window comparator, threshold
/// registers, and statistics (min/max/recent): ADS7x38 and ADS7x28.
pub trait HasStats: Chip {}
impl HasStats for Ads7x38 {}
impl HasStats for Ads7x28 {}

/// Implemented by chips with RMS computation and zero-crossing detection:
/// ADS7x28 only.
pub trait HasRmsZcd: HasStats {}
impl HasRmsZcd for Ads7x28 {}

// ── ZCD GPO value type ────────────────────────────────────────────────────────

/// GPO output value applied on a zero-crossing detection edge (rise/fall).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ZcdGpoValue {
    Rise0Fall0 = 0,
    Rise0Fall1 = 1,
    Rise1Fall0 = 2,
    Rise1Fall1 = 3,
}

impl From<u8> for ZcdGpoValue {
    fn from(v: u8) -> Self {
        match v & 0x3 {
            0 => Self::Rise0Fall0,
            1 => Self::Rise0Fall1,
            2 => Self::Rise1Fall0,
            _ => Self::Rise1Fall1,
        }
    }
}

impl From<ZcdGpoValue> for u8 {
    fn from(v: ZcdGpoValue) -> Self {
        v as u8
    }
}

// ── Register map ──────────────────────────────────────────────────────────────

device_driver::create_device!(device_name: Device, manifest: "device.yaml");

// ── I2C address ───────────────────────────────────────────────────────────────

/// I2C address as set by the R1/R2 resistors on the ADDR pin.
/// See datasheet Table 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Address {
    /// R1 = DNP,    R2 = DNP   (default)
    X10 = 0x10,
    /// R1 = DNP,    R2 = 11 kΩ
    X11 = 0x11,
    /// R1 = DNP,    R2 = 33 kΩ
    X12 = 0x12,
    /// R1 = DNP,    R2 = 100 kΩ
    X13 = 0x13,
    /// R1 = 100 kΩ, R2 = DNP
    X14 = 0x14,
    /// R1 = 33 kΩ,  R2 = DNP
    X15 = 0x15,
    /// R1 = 11 kΩ,  R2 = DNP
    X16 = 0x16,
    /// R1 = 0 Ω,    R2 = DNP
    X17 = 0x17,
}

// ── Bus interfaces ────────────────────────────────────────────────────────────

const OPCODE_READ: u8 = 0x10;
const OPCODE_WRITE: u8 = 0x08;
/// Register address for MANUAL_CH_SEL (§7.4.2 manual mode channel selection).
const REG_MANUAL_CH_SEL: u8 = 0x11;

/// I²C bus interface. Used by TLA2528, ADS7138, ADS7128.
pub struct I2cInterface<BUS> {
    i2c: BUS,
    addr: Address,
}

impl<BUS: I2c> AsyncRegisterInterface for I2cInterface<BUS> {
    type Error = BUS::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write(self.addr as u8, &[OPCODE_WRITE, address, data[0]])
            .await
    }

    async fn read_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c
            .write_read(self.addr as u8, &[OPCODE_READ, address], data)
            .await
    }
}

/// SPI bus interface. Used by TLA2518, ADS7038, ADS7038H, ADS7028.
pub struct SpiInterface<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> AsyncRegisterInterface for SpiInterface<SPI> {
    type Error = SPI::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        // 24-bit write frame: [opcode, address, data] (§7.3.12.2)
        self.spi.write(&[OPCODE_WRITE, address, data[0]]).await
    }

    async fn read_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Two-frame register read (§7.3.12.2):
        // Frame 1: issue read command; device latches requested register on CS rise.
        self.spi.write(&[OPCODE_READ, address, 0x00]).await?;
        // Frame 2: first SDO byte is the register data.
        self.spi.read(&mut data[..1]).await
    }
}

// ── CanConvert: raw ADC output frame read ─────────────────────────────────────
//
// Sealed trait implemented by both interfaces. Provides the bus-level primitive
// that `read_chN` dispatches through.
//
// Return value is the raw 16-bit MSB-aligned ADC word:
//   OSR = 0 (no averaging): 12-bit result in bits [15:4]; bits [3:0] = 0.
//   OSR > 0 (averaging):    full 16-bit result in bits [15:0].

#[allow(async_fn_in_trait)]
pub trait CanConvert: AsyncRegisterInterface<AddressType = u8> {
    async fn read_conversion_raw(&mut self, channel: u8) -> Result<u16, Self::Error>;
}

impl<BUS: I2c> CanConvert for I2cInterface<BUS> {
    async fn read_conversion_raw(&mut self, channel: u8) -> Result<u16, BUS::Error> {
        // Write MANUAL_CH_SEL so the device converts the right channel.
        self.write_register(REG_MANUAL_CH_SEL, 8, &[channel & 0x0F])
            .await?;
        // Bare I²C read: device stretches SCL while converting (~1.17 µs @ 1 MHz).
        let mut buf = [0u8; 2];
        self.i2c.read(self.addr as u8, &mut buf).await?;
        Ok((buf[0] as u16) << 8 | buf[1] as u16)
    }
}

impl<SPI: SpiDevice> CanConvert for SpiInterface<SPI> {
    async fn read_conversion_raw(&mut self, channel: u8) -> Result<u16, SPI::Error> {
        // On-the-fly mode (§7.4.3): channel ID in the first 5 SDI bits.
        // ADC samples the newly selected channel on the CS edge — zero latency.
        let mut buf = [(channel & 0x1F) << 3, 0x00];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok((buf[0] as u16) << 8 | buf[1] as u16)
    }
}

// ── Channel mode types ────────────────────────────────────────────────────────

pub struct Unconfigured;
pub struct AnalogIn;
pub struct DigitalIn;
pub struct DigitalOut<DRIVE>(core::marker::PhantomData<DRIVE>);
pub struct OpenDrain;
pub struct PushPull;

// ── Driver struct ─────────────────────────────────────────────────────────────

pub struct Driver<
    IF,
    CHIP,
    C0 = Unconfigured,
    C1 = Unconfigured,
    C2 = Unconfigured,
    C3 = Unconfigured,
    C4 = Unconfigured,
    C5 = Unconfigured,
    C6 = Unconfigured,
    C7 = Unconfigured,
> {
    pub device: Device<IF>,
    _chip: core::marker::PhantomData<CHIP>,
    _channels: core::marker::PhantomData<(C0, C1, C2, C3, C4, C5, C6, C7)>,
}

impl<BUS: I2c, CHIP: Chip> Driver<I2cInterface<BUS>, CHIP> {
    /// Create a new I²C driver. All channels start as `Unconfigured`.
    pub fn new(i2c: BUS, addr: Address) -> Self {
        Self {
            device: Device::new(I2cInterface { i2c, addr }),
            _chip: core::marker::PhantomData,
            _channels: core::marker::PhantomData,
        }
    }
}

impl<SPI: SpiDevice, CHIP: Chip> Driver<SpiInterface<SPI>, CHIP> {
    /// Create a new SPI driver. All channels start as `Unconfigured`.
    pub fn new(spi: SPI) -> Self {
        Self {
            device: Device::new(SpiInterface { spi }),
            _chip: core::marker::PhantomData,
            _channels: core::marker::PhantomData,
        }
    }
}

// ── Global methods: all chips ─────────────────────────────────────────────────

impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: Chip, C0, C1, C2, C3, C4, C5, C6, C7>
    Driver<IF, CHIP, C0, C1, C2, C3, C4, C5, C6, C7>
{
    /// Reset all registers to defaults. Consumes the driver and returns a fresh
    /// instance with all channels in the `Unconfigured` state.
    pub async fn reset(mut self) -> Result<Driver<IF, CHIP>, IF::Error> {
        self.device
            .general_cfg()
            .write_with_zero_async(|r| r.set_rst(true))
            .await?;
        Ok(Driver {
            device: self.device,
            _chip: core::marker::PhantomData,
            _channels: core::marker::PhantomData,
        })
    }

    /// Initiate ADC offset calibration. The `cal` bit auto-clears when complete.
    pub async fn calibrate(&mut self) -> Result<(), IF::Error> {
        self.device
            .general_cfg()
            .modify_async(|r| r.set_cal(true))
            .await?;
        Ok(())
    }

    /// Clear the brown-out reset indicator (W1C).
    pub async fn clear_bor(&mut self) -> Result<(), IF::Error> {
        self.device
            .system_status()
            .write_with_zero_async(|r| r.set_bor(true))
            .await?;
        Ok(())
    }

    /// Set the oversampling ratio applied to every conversion.
    pub async fn set_oversampling(&mut self, ratio: OversamplingRatio) -> Result<(), IF::Error> {
        self.device
            .osr_cfg()
            .modify_async(|r| r.set_osr(ratio))
            .await?;
        Ok(())
    }
}

// ── Global methods: HasStats chips (ADS7x38, ADS7x28) ────────────────────────

impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: HasStats, C0, C1, C2, C3, C4, C5, C6, C7>
    Driver<IF, CHIP, C0, C1, C2, C3, C4, C5, C6, C7>
{
    /// Enable the digital window comparator.
    pub async fn enable_dwc(&mut self) -> Result<(), IF::Error> {
        self.device
            .general_cfg()
            .modify_async(|r| r.set_dwc_en(true))
            .await?;
        Ok(())
    }

    /// Disable the digital window comparator.
    pub async fn disable_dwc(&mut self) -> Result<(), IF::Error> {
        self.device
            .general_cfg()
            .modify_async(|r| r.set_dwc_en(false))
            .await?;
        Ok(())
    }

    /// Configure the ALERT pin driver type and assertion logic.
    pub async fn configure_alert_pin(
        &mut self,
        push_pull: bool,
        logic: AlertLogic,
    ) -> Result<(), IF::Error> {
        self.device
            .alert_pin_cfg()
            .write_with_zero_async(|r| {
                r.set_alert_drive(push_pull);
                r.set_alert_logic(logic);
            })
            .await?;
        Ok(())
    }

    /// Set which channels can assert the ALERT pin (bit N = 1 enables CH[N]).
    pub async fn set_alert_channel_mask(&mut self, mask: u8) -> Result<(), IF::Error> {
        self.device
            .alert_ch_sel()
            .write_with_zero_async(|r| r.set_alert_ch_sel(mask))
            .await?;
        Ok(())
    }

    /// Read the combined event flags register (bit N = OR of high and low flags for CH[N]).
    pub async fn event_flags(&mut self) -> Result<u8, IF::Error> {
        Ok(self.device.event_flag().read_async().await?.event_flag())
    }

    /// Read the high-threshold (or GPIO logic-1) event flags.
    pub async fn event_high_flags(&mut self) -> Result<u8, IF::Error> {
        Ok(self
            .device
            .event_high_flag()
            .read_async()
            .await?
            .event_high_flag())
    }

    /// Read the low-threshold (or GPIO logic-0) event flags.
    pub async fn event_low_flags(&mut self) -> Result<u8, IF::Error> {
        Ok(self
            .device
            .event_low_flag()
            .read_async()
            .await?
            .event_low_flag())
    }

    /// Clear high-event flags for the given channel bitmask (W1C).
    pub async fn clear_event_high_flags(&mut self, mask: u8) -> Result<(), IF::Error> {
        self.device
            .event_high_flag()
            .write_with_zero_async(|r| r.set_event_high_flag(mask))
            .await?;
        Ok(())
    }

    /// Clear low-event flags for the given channel bitmask (W1C).
    pub async fn clear_event_low_flags(&mut self, mask: u8) -> Result<(), IF::Error> {
        self.device
            .event_low_flag()
            .write_with_zero_async(|r| r.set_event_low_flag(mask))
            .await?;
        Ok(())
    }
}

// ── Global methods: HasRmsZcd chips (ADS7x28) ────────────────────────────────

impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: HasRmsZcd, C0, C1, C2, C3, C4, C5, C6, C7>
    Driver<IF, CHIP, C0, C1, C2, C3, C4, C5, C6, C7>
{
    /// Configure the RMS computation module.
    ///
    /// `channel` selects which AIN is measured; `dc_sub` subtracts the DC
    /// component before computing; `samples` sets the accumulation window.
    /// Call `enable_dwc()` (which also enables the statistics module) before
    /// reading results.
    pub async fn configure_rms(
        &mut self,
        channel: RmsChannelId,
        dc_sub: bool,
        samples: RmsSampleCount,
    ) -> Result<(), IF::Error> {
        self.device
            .rms_cfg()
            .write_with_zero_async(|r| {
                r.set_rms_chid(channel);
                r.set_rms_dc_sub(dc_sub);
                r.set_rms_samples(samples);
            })
            .await?;
        Ok(())
    }

    /// Read the 16-bit RMS result.
    pub async fn read_rms(&mut self) -> Result<u16, IF::Error> {
        let lsb = self.device.rms_lsb().read_async().await?.value();
        let msb = self.device.rms_msb().read_async().await?.value();
        Ok(((msb as u16) << 8) | lsb as u16)
    }

    /// Configure the zero-crossing detection blanking time.
    ///
    /// `multiplier_8x`: false = 1× blanking, true = 8× blanking.
    /// `blanking`: 7-bit blanking count value.
    pub async fn configure_zcd_blanking(
        &mut self,
        multiplier_8x: bool,
        blanking: u8,
    ) -> Result<(), IF::Error> {
        self.device
            .zcd_blanking_cfg()
            .write_with_zero_async(|r| {
                r.set_mult_en(multiplier_8x);
                r.set_zcd_blanking(blanking & 0x7F);
            })
            .await?;
        Ok(())
    }

    /// Set the GPO value driven on ZCD edge detection for channels 0–3.
    pub async fn set_zcd_gpo_ch0_ch3(
        &mut self,
        ch0: ZcdGpoValue,
        ch1: ZcdGpoValue,
        ch2: ZcdGpoValue,
        ch3: ZcdGpoValue,
    ) -> Result<(), IF::Error> {
        self.device
            .gpo_value_zcd_cfg_ch_0_ch_3()
            .write_with_zero_async(|r| {
                r.set_ch_0(ch0);
                r.set_ch_1(ch1);
                r.set_ch_2(ch2);
                r.set_ch_3(ch3);
            })
            .await?;
        Ok(())
    }

    /// Set the GPO value driven on ZCD edge detection for channels 4–7.
    pub async fn set_zcd_gpo_ch4_ch7(
        &mut self,
        ch4: ZcdGpoValue,
        ch5: ZcdGpoValue,
        ch6: ZcdGpoValue,
        ch7: ZcdGpoValue,
    ) -> Result<(), IF::Error> {
        self.device
            .gpo_value_zcd_cfg_ch_4_ch_7()
            .write_with_zero_async(|r| {
                r.set_ch_4(ch4);
                r.set_ch_5(ch5);
                r.set_ch_6(ch6);
                r.set_ch_7(ch7);
            })
            .await?;
        Ok(())
    }

    /// Set which GPO outputs are updated on zero-crossing detection (bit N = 1 enables GPO[N]).
    pub async fn set_zcd_gpo_update_mask(&mut self, mask: u8) -> Result<(), IF::Error> {
        self.device
            .gpo_zcd_update_en()
            .write_with_zero_async(|r| r.set_gpo_zcd_update_en(mask))
            .await?;
        Ok(())
    }
}

// ── Per-channel typestate impl ────────────────────────────────────────────────

/// Generates configure/read/write impls for one channel.
///
/// Arguments:
///   $n    — channel index literal (0–7)
///   $pre  — channel type params that come *before* channel $n in the struct
///   $post — channel type params that come *after* channel $n in the struct
macro_rules! impl_channel {
    ($n:literal, ($($pre:ident),*), ($($post:ident),*)) => {
        pastey::paste! {
            // ── configure (from any current mode, any chip) ───────────────

            impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: Chip, [<C $n>] $(, $pre)* $(, $post)*>
                Driver<IF, CHIP, $($pre,)* [<C $n>], $($post,)*>
            {
                pub async fn [<configure_ch $n _as_analog>](mut self)
                    -> Result<Driver<IF, CHIP, $($pre,)* AnalogIn, $($post,)*>, IF::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() & !(1u8 << $n))
                    }).await?;
                    Ok(Driver { device: self.device, _chip: core::marker::PhantomData, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_in>](mut self)
                    -> Result<Driver<IF, CHIP, $($pre,)* DigitalIn, $($post,)*>, IF::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpio_cfg().modify_async(|r| {
                        r.set_gpio_cfg(r.gpio_cfg() & !(1u8 << $n))
                    }).await?;
                    Ok(Driver { device: self.device, _chip: core::marker::PhantomData, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_out_push_pull>](mut self)
                    -> Result<Driver<IF, CHIP, $($pre,)* DigitalOut<PushPull>, $($post,)*>, IF::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpio_cfg().modify_async(|r| {
                        r.set_gpio_cfg(r.gpio_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpo_drive_cfg().modify_async(|r| {
                        r.set_gpo_drive_cfg(r.gpo_drive_cfg() | (1u8 << $n))
                    }).await?;
                    Ok(Driver { device: self.device, _chip: core::marker::PhantomData, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_out_open_drain>](mut self)
                    -> Result<Driver<IF, CHIP, $($pre,)* DigitalOut<OpenDrain>, $($post,)*>, IF::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpio_cfg().modify_async(|r| {
                        r.set_gpio_cfg(r.gpio_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpo_drive_cfg().modify_async(|r| {
                        r.set_gpo_drive_cfg(r.gpo_drive_cfg() & !(1u8 << $n))
                    }).await?;
                    Ok(Driver { device: self.device, _chip: core::marker::PhantomData, _channels: core::marker::PhantomData })
                }
            }

            // ── AnalogIn: SCL-stretch (I²C) / on-the-fly (SPI) read ──────
            //
            // Available on all chips. For I²C, the device stretches SCL during
            // conversion. For SPI, uses on-the-fly mode (§7.4.3) — no latency.
            //
            // Returns the raw 16-bit MSB-aligned ADC word:
            //   OSR = 0: 12-bit result in bits [15:4]; bits [3:0] = 0.
            //   OSR > 0: full 16-bit averaged result in bits [15:0].

            impl<IF: CanConvert, CHIP: Chip $(, $pre)* $(, $post)*>
                Driver<IF, CHIP, $($pre,)* AnalogIn, $($post,)*>
            {
                pub async fn [<read_ch $n>](&mut self) -> Result<u16, IF::Error> {
                    self.device.interface.read_conversion_raw($n).await
                }
            }

            // ── AnalogIn: polled read + thresholds (HasStats chips only) ──

            impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: HasStats $(, $pre)* $(, $post)*>
                Driver<IF, CHIP, $($pre,)* AnalogIn, $($post,)*>
            {
                /// Trigger a conversion on channel $n without SCL stretching and
                /// return the raw 16-bit MSB-aligned ADC word.
                ///
                /// Enables STATS_EN, triggers the conversion via CNVST, polls
                /// OSR_DONE, then reads the result from the RECENT_CH$n registers.
                ///
                /// Returns the raw 16-bit MSB-aligned ADC word:
                ///   OSR = 0: 12-bit result in bits [15:4]; bits [3:0] = 0.
                ///   OSR > 0: full 16-bit averaged result in bits [15:0].
                pub async fn [<read_ch $n _polled>](&mut self) -> Result<u16, IF::Error> {
                    self.device.general_cfg().modify_async(|r| r.set_stats_en(true)).await?;
                    self.device.manual_ch_sel().write_async(|r| {
                        r.set_manual_chid(ChannelId::[<Ain $n>])
                    }).await?;
                    self.device.general_cfg().modify_async(|r| r.set_cnvst(true)).await?;
                    loop {
                        if self.device.system_status().read_async().await?.osr_done() {
                            break;
                        }
                    }
                    self.device.system_status().write_with_zero_async(|r| {
                        r.set_osr_done(true)
                    }).await?;
                    let lsb = self.device.recent_ch_lsb($n).read_async().await?.value();
                    let msb = self.device.recent_ch_msb($n).read_async().await?.value();
                    Ok(((msb as u16) << 8) | lsb as u16)
                }

                /// Set the high and low alert thresholds for channel $n (12-bit, 0–4095).
                pub async fn [<set_ch $n _thresholds>](
                    &mut self,
                    high: u16,
                    low: u16,
                ) -> Result<(), IF::Error> {
                    self.device
                        .high_th_ch($n)
                        .write_with_zero_async(|r| r.set_high_threshold_msb((high >> 4) as u8))
                        .await?;
                    self.device
                        .hysteresis_ch($n)
                        .modify_async(|r| r.set_high_threshold_lsb((high & 0xF) as u8))
                        .await?;
                    self.device
                        .low_th_ch($n)
                        .write_with_zero_async(|r| r.set_low_threshold_msb((low >> 4) as u8))
                        .await?;
                    self.device
                        .event_count_ch($n)
                        .modify_async(|r| r.set_low_threshold_lsb((low & 0xF) as u8))
                        .await?;
                    Ok(())
                }

                /// Set the hysteresis for channel $n (4-bit; effective value = `hyst << 3` LSBs).
                pub async fn [<set_ch $n _hysteresis>](
                    &mut self,
                    hyst: u8,
                ) -> Result<(), IF::Error> {
                    self.device
                        .hysteresis_ch($n)
                        .modify_async(|r| r.set_hysteresis(hyst & 0xF))
                        .await?;
                    Ok(())
                }

                /// Set the event count for channel $n (4-bit; alert fires after `count + 1`
                /// consecutive threshold violations).
                pub async fn [<set_ch $n _event_count>](
                    &mut self,
                    count: u8,
                ) -> Result<(), IF::Error> {
                    self.device
                        .event_count_ch($n)
                        .modify_async(|r| r.set_event_count(count & 0xF))
                        .await?;
                    Ok(())
                }
            }

            // ── DigitalIn: read (all chips) ───────────────────────────────

            impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: Chip $(, $pre)* $(, $post)*>
                Driver<IF, CHIP, $($pre,)* DigitalIn, $($post,)*>
            {
                pub async fn [<is_ch $n _high>](&mut self) -> Result<bool, IF::Error> {
                    Ok(self.device.gpi_value().read_async().await?.gpi_value() & (1u8 << $n) != 0)
                }
            }

            // ── DigitalOut: write (all chips) ─────────────────────────────

            impl<IF: AsyncRegisterInterface<AddressType = u8>, CHIP: Chip, D $(, $pre)* $(, $post)*>
                Driver<IF, CHIP, $($pre,)* DigitalOut<D>, $($post,)*>
            {
                pub async fn [<write_ch $n>](&mut self, high: bool) -> Result<(), IF::Error> {
                    self.device.gpo_value().modify_async(|r| {
                        let v = r.gpo_value();
                        r.set_gpo_value(if high { v | (1u8 << $n) } else { v & !(1u8 << $n) })
                    }).await?;
                    Ok(())
                }
            }
        }
    }
}

impl_channel!(0, (), (C1, C2, C3, C4, C5, C6, C7));
impl_channel!(1, (C0), (C2, C3, C4, C5, C6, C7));
impl_channel!(2, (C0, C1), (C3, C4, C5, C6, C7));
impl_channel!(3, (C0, C1, C2), (C4, C5, C6, C7));
impl_channel!(4, (C0, C1, C2, C3), (C5, C6, C7));
impl_channel!(5, (C0, C1, C2, C3, C4), (C6, C7));
impl_channel!(6, (C0, C1, C2, C3, C4, C5), (C7));
impl_channel!(7, (C0, C1, C2, C3, C4, C5, C6), ());

// ── Per-part-number type aliases ──────────────────────────────────────────────

macro_rules! chip_alias {
    ($bus:ident : $(#[$attr:meta])* $name:ident, $chip:ty) => {
        pastey::paste! {
            $(#[$attr])*
            pub type $name<
                BUS,
                C0 = Unconfigured,
                C1 = Unconfigured,
                C2 = Unconfigured,
                C3 = Unconfigured,
                C4 = Unconfigured,
                C5 = Unconfigured,
                C6 = Unconfigured,
                C7 = Unconfigured,
            > = Driver<[<$bus:camel Interface>]<BUS>, $chip, C0, C1, C2, C3, C4, C5, C6, C7>;
        }
    };
}

chip_alias!(i2c: Tla2528, Tla252x);
chip_alias!(spi: Tla2518, Tla252x);
chip_alias!(i2c: Ads7138, Ads7x38);
chip_alias!(spi: Ads7038, Ads7x38);
chip_alias!(spi:
    /// ADS7038H is a higher-throughput (1.5 MSPS vs 1 MSPS) SPI variant of the ADS7038.
    /// The register map and feature set are identical; the speed difference is a hardware
    /// characteristic with no register-level impact.
    Ads7038H,
    Ads7x38
);
chip_alias!(i2c: Ads7128, Ads7x28);
chip_alias!(spi: Ads7028, Ads7x28);
