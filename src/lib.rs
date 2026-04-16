#![cfg_attr(not(test), no_std)]

use device_driver::AsyncRegisterInterface;
use embedded_hal_async::i2c::I2c;

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

// ── DeviceInterface ───────────────────────────────────────────────────────────

pub struct DeviceInterface<BUS> {
    i2c: BUS,
    addr: Address,
}

// ADS7138 I2C single-register opcodes
const OPCODE_READ: u8 = 0x10;
const OPCODE_WRITE: u8 = 0x08;

impl<BUS: I2c> DeviceInterface<BUS> {
    /// Raw I2C read that triggers an ADC conversion.
    ///
    /// The ADS7138 stretches SCL while converting (~1.17 µs @ 1 MHz).
    /// Channel must be selected via MANUAL_CH_SEL before calling this.
    /// Returns the raw 16-bit output word; bits [15:4] hold the 12-bit result MSB-aligned.
    pub(crate) async fn read_conversion_raw(&mut self) -> Result<u16, BUS::Error> {
        let mut buf = [0u8; 2];
        self.i2c.read(self.addr as u8, &mut buf).await?;
        Ok((buf[0] as u16) << 8 | buf[1] as u16)
    }
}

impl<BUS: I2c> AsyncRegisterInterface for DeviceInterface<BUS> {
    type Error = BUS::Error;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write(self.addr as u8, &[OPCODE_WRITE, address, data[0]]).await
    }

    async fn read_register(
        &mut self,
        address: u8,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.i2c.write_read(self.addr as u8, &[OPCODE_READ, address], data).await
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

pub struct Ads7138<
    BUS,
    C0 = Unconfigured,
    C1 = Unconfigured,
    C2 = Unconfigured,
    C3 = Unconfigured,
    C4 = Unconfigured,
    C5 = Unconfigured,
    C6 = Unconfigured,
    C7 = Unconfigured,
> {
    pub device: Device<DeviceInterface<BUS>>,
    _channels: core::marker::PhantomData<(C0, C1, C2, C3, C4, C5, C6, C7)>,
}

impl<BUS: I2c> Ads7138<BUS> {
    pub fn new(i2c: BUS, addr: Address) -> Self {
        Self {
            device: Device::new(DeviceInterface { i2c, addr }),
            _channels: core::marker::PhantomData,
        }
    }
}

// ── Global device methods ─────────────────────────────────────────────────────

impl<BUS: I2c, C0, C1, C2, C3, C4, C5, C6, C7>
    Ads7138<BUS, C0, C1, C2, C3, C4, C5, C6, C7>
{
    /// Reset all registers to defaults. Consumes the driver and returns a fresh
    /// instance with all channels in the `Unconfigured` state.
    pub async fn reset(mut self) -> Result<Ads7138<BUS>, BUS::Error> {
        self.device
            .general_cfg()
            .write_with_zero_async(|r| r.set_rst(true))
            .await?;
        Ok(Ads7138 { device: self.device, _channels: core::marker::PhantomData })
    }

    /// Initiate ADC offset calibration. The `cal` bit auto-clears when complete.
    pub async fn calibrate(&mut self) -> Result<(), BUS::Error> {
        self.device.general_cfg().modify_async(|r| r.set_cal(true)).await?;
        Ok(())
    }

    /// Clear the brown-out reset indicator (W1C).
    pub async fn clear_bor(&mut self) -> Result<(), BUS::Error> {
        self.device
            .system_status()
            .write_with_zero_async(|r| r.set_bor(true))
            .await?;
        Ok(())
    }

    /// Set the oversampling ratio applied to every conversion.
    pub async fn set_oversampling(
        &mut self,
        ratio: OversamplingRatio,
    ) -> Result<(), BUS::Error> {
        self.device.osr_cfg().modify_async(|r| r.set_osr(ratio)).await?;
        Ok(())
    }

    /// Enable the digital window comparator.
    pub async fn enable_dwc(&mut self) -> Result<(), BUS::Error> {
        self.device.general_cfg().modify_async(|r| r.set_dwc_en(true)).await?;
        Ok(())
    }

    /// Disable the digital window comparator.
    pub async fn disable_dwc(&mut self) -> Result<(), BUS::Error> {
        self.device.general_cfg().modify_async(|r| r.set_dwc_en(false)).await?;
        Ok(())
    }

    /// Configure the ALERT pin driver type and assertion logic.
    pub async fn configure_alert_pin(
        &mut self,
        push_pull: bool,
        logic: AlertLogic,
    ) -> Result<(), BUS::Error> {
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
    pub async fn set_alert_channel_mask(&mut self, mask: u8) -> Result<(), BUS::Error> {
        self.device
            .alert_ch_sel()
            .write_with_zero_async(|r| r.set_alert_ch_sel(mask))
            .await?;
        Ok(())
    }

    /// Read the combined event flags register (bit N = OR of high and low flags for CH[N]).
    pub async fn event_flags(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.device.event_flag().read_async().await?.event_flag())
    }

    /// Read the high-threshold (or GPIO logic-1) event flags.
    pub async fn event_high_flags(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.device.event_high_flag().read_async().await?.event_high_flag())
    }

    /// Read the low-threshold (or GPIO logic-0) event flags.
    pub async fn event_low_flags(&mut self) -> Result<u8, BUS::Error> {
        Ok(self.device.event_low_flag().read_async().await?.event_low_flag())
    }

    /// Clear high-event flags for the given channel bitmask (W1C).
    pub async fn clear_event_high_flags(&mut self, mask: u8) -> Result<(), BUS::Error> {
        self.device
            .event_high_flag()
            .write_with_zero_async(|r| r.set_event_high_flag(mask))
            .await?;
        Ok(())
    }

    /// Clear low-event flags for the given channel bitmask (W1C).
    pub async fn clear_event_low_flags(&mut self, mask: u8) -> Result<(), BUS::Error> {
        self.device
            .event_low_flag()
            .write_with_zero_async(|r| r.set_event_low_flag(mask))
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
            // ── configure (from any current mode) ────────────────────────────

            impl<BUS: I2c, [<C $n>] $(, $pre)* $(, $post)*>
                Ads7138<BUS, $($pre,)* [<C $n>], $($post,)*>
            {
                pub async fn [<configure_ch $n _as_analog>](mut self)
                    -> Result<Ads7138<BUS, $($pre,)* AnalogIn, $($post,)*>, BUS::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() & !(1u8 << $n))
                    }).await?;
                    Ok(Ads7138 { device: self.device, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_in>](mut self)
                    -> Result<Ads7138<BUS, $($pre,)* DigitalIn, $($post,)*>, BUS::Error>
                {
                    self.device.pin_cfg().modify_async(|r| {
                        r.set_pin_cfg(r.pin_cfg() | (1u8 << $n))
                    }).await?;
                    self.device.gpio_cfg().modify_async(|r| {
                        r.set_gpio_cfg(r.gpio_cfg() & !(1u8 << $n))
                    }).await?;
                    Ok(Ads7138 { device: self.device, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_out_push_pull>](mut self)
                    -> Result<Ads7138<BUS, $($pre,)* DigitalOut<PushPull>, $($post,)*>, BUS::Error>
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
                    Ok(Ads7138 { device: self.device, _channels: core::marker::PhantomData })
                }

                pub async fn [<configure_ch $n _as_digital_out_open_drain>](mut self)
                    -> Result<Ads7138<BUS, $($pre,)* DigitalOut<OpenDrain>, $($post,)*>, BUS::Error>
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
                    Ok(Ads7138 { device: self.device, _channels: core::marker::PhantomData })
                }
            }

            // ── AnalogIn: read ────────────────────────────────────────────────

            impl<BUS: I2c $(, $pre)* $(, $post)*>
                Ads7138<BUS, $($pre,)* AnalogIn, $($post,)*>
            {
                /// Trigger a conversion on channel $n via SCL stretching and return
                /// the 12-bit result (0–4095).
                ///
                /// Note: SCL stretching can cause issues on some host controllers.
                /// Prefer [`read_ch$n_polled`] if your I2C controller does not
                /// support clock stretching.
                pub async fn [<read_ch $n>](&mut self) -> Result<u16, BUS::Error> {
                    self.device.manual_ch_sel().write_async(|r| {
                        r.set_manual_chid(ChannelId::[<Ain $n>])
                    }).await?;
                    let raw = self.device.interface.read_conversion_raw().await?;
                    Ok(raw >> 4)
                }

                /// Trigger a conversion on channel $n without SCL stretching and
                /// return the 12-bit result (0–4095).
                ///
                /// Enables STATS_EN (clearing the statistics registers), triggers
                /// the conversion via CNVST, polls OSR_DONE, then reads the result
                /// from the RECENT_CH$n registers.
                pub async fn [<read_ch $n _polled>](&mut self) -> Result<u16, BUS::Error> {
                    // Enable statistics and select the channel
                    self.device.general_cfg().modify_async(|r| r.set_stats_en(true)).await?;
                    self.device.manual_ch_sel().write_async(|r| {
                        r.set_manual_chid(ChannelId::[<Ain $n>])
                    }).await?;
                    // Trigger conversion (no SCL stretching)
                    self.device.general_cfg().modify_async(|r| r.set_cnvst(true)).await?;
                    // Poll until the conversion (and any oversampling) is complete
                    loop {
                        if self.device.system_status().read_async().await?.osr_done() {
                            break;
                        }
                    }
                    // Clear the OSR_DONE flag (W1C)
                    self.device.system_status().write_with_zero_async(|r| {
                        r.set_osr_done(true)
                    }).await?;
                    // Read the 12-bit result from the RECENT registers
                    // MSB register holds bits [11:4]; upper nibble of LSB register holds bits [3:0]
                    let lsb = self.device.recent_ch_lsb($n).read_async().await?.value();
                    let msb = self.device.recent_ch_msb($n).read_async().await?.value();
                    Ok(((msb as u16) << 4) | ((lsb as u16) >> 4))
                }

                /// Set the high and low alert thresholds for channel $n (12-bit, 0–4095).
                ///
                /// The 12-bit value is split: upper 8 bits go to the `*_msb` register and
                /// lower 4 bits are packed into the upper nibble of the adjacent register.
                pub async fn [<set_ch $n _thresholds>](
                    &mut self,
                    high: u16,
                    low: u16,
                ) -> Result<(), BUS::Error> {
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
                ) -> Result<(), BUS::Error> {
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
                ) -> Result<(), BUS::Error> {
                    self.device
                        .event_count_ch($n)
                        .modify_async(|r| r.set_event_count(count & 0xF))
                        .await?;
                    Ok(())
                }
            }

            // ── DigitalIn: read ───────────────────────────────────────────────

            impl<BUS: I2c $(, $pre)* $(, $post)*>
                Ads7138<BUS, $($pre,)* DigitalIn, $($post,)*>
            {
                pub async fn [<is_ch $n _high>](&mut self) -> Result<bool, BUS::Error> {
                    Ok(self.device.gpi_value().read_async().await?.gpi_value() & (1u8 << $n) != 0)
                }
            }

            // ── DigitalOut: write (open-drain or push-pull) ───────────────────

            impl<BUS: I2c, D $(, $pre)* $(, $post)*>
                Ads7138<BUS, $($pre,)* DigitalOut<D>, $($post,)*>
            {
                pub async fn [<write_ch $n>](&mut self, high: bool) -> Result<(), BUS::Error> {
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

impl_channel!(0, (),                         (C1, C2, C3, C4, C5, C6, C7));
impl_channel!(1, (C0),                       (C2, C3, C4, C5, C6, C7));
impl_channel!(2, (C0, C1),                   (C3, C4, C5, C6, C7));
impl_channel!(3, (C0, C1, C2),               (C4, C5, C6, C7));
impl_channel!(4, (C0, C1, C2, C3),           (C5, C6, C7));
impl_channel!(5, (C0, C1, C2, C3, C4),       (C6, C7));
impl_channel!(6, (C0, C1, C2, C3, C4, C5),   (C7));
impl_channel!(7, (C0, C1, C2, C3, C4, C5, C6), ());
