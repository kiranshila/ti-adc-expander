# ti-adc-expander

An async Rust driver for the TI family of 12-bit, 8-channel ADC + GPIO expander chips:

| Part     | Interface | Max rate | Chip type  | DWC / Alerts / Stats | RMS | ZCD |
|----------|-----------|----------|------------|:--------------------:|:---:|:---:|
| TLA2518  | SPI       | 1 MSPS   | `Tla252x`  |                      |     |     |
| TLA2528  | I²C       | 140 kSPS | `Tla252x`  |                      |     |     |
| ADS7038  | SPI       | 1 MSPS   | `Ads7x38`  | ✓                    |     |     |
| ADS7038H | SPI       | 1.5 MSPS | `Ads7x38`  | ✓                    |     |     |
| ADS7138  | I²C       | 140 kSPS | `Ads7x38`  | ✓                    |     |     |
| ADS7028  | SPI       | 1 MSPS   | `Ads7x28`  | ✓                    | ✓   | ✓   |
| ADS7128  | I²C       | 140 kSPS | `Ads7x28`  | ✓                    | ✓   | ✓   |

All chips are type-aliased (`Ads7138<BUS>`, `Tla2528<BUS>`, etc.) over a common
`Driver<BUS, CHIP, C0..C7>` struct. Features unavailable on a given chip are
gated at compile time via trait bounds — calling `read_ch0_polled()` on a
`Tla252x` is a type error.

Built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) and
[`device-driver`](https://crates.io/crates/device-driver). All methods return the
bus error type directly — no wrapper enum.

> **Note:** SPI variants (TLA2518, ADS7038, ADS7028) share the same register map
> as their I²C counterparts but require a different `DeviceInterface`
> implementation. I²C support is complete; SPI support is planned.

## Features

- Async-only, `no_std` compatible
- Compile-time chip capability gating via sealed trait hierarchy
- Typestate-enforced channel configuration — analog / digital-in / digital-out
  modes are verified at compile time
- Two analog read strategies (ADS7x38 / ADS7x28):
  - SCL-stretch read (one I²C transaction, uses clock stretching)
  - Polled read (no clock stretching; uses `CNVST` + `OSR_DONE` polling)
- TLA2528 / TLA2518: SCL-stretch read only (no statistics registers)
- Full register access via the `device` field for anything not covered by the API
- Optional `defmt` support via the `defmt` feature

## Cargo.toml

```toml
[dependencies]
ti-adc-expander = "0.1"
```

To enable `defmt`:

```toml
ti-adc-expander = { version = "0.1", features = ["defmt"] }
```

## Chip capability tiers

```
Tla252x  (TLA2518 / TLA2528)
│  ADC reads (SCL-stretch), GPIO, oversampling, sequencing
│
└── Ads7x38  (ADS7038 / ADS7138)          [+ HasStats]
    │  Polled ADC reads, digital window comparator,
    │  per-channel alert thresholds, min/max/recent statistics,
    │  ALERT pin, GPO trigger on events
    │
    └── Ads7x28  (ADS7028 / ADS7128)      [+ HasRmsZcd]
           RMS computation, zero-crossing detection,
           ZCD-linked GPO output control
```

## I²C address

The device address is set by resistors on the `ADDR` pin (I²C parts only):

| Variant | Address | R1       | R2       |
|---------|---------|----------|----------|
| `X10`   | `0x10`  | DNP      | DNP      |
| `X11`   | `0x11`  | DNP      | 11 kΩ    |
| `X12`   | `0x12`  | DNP      | 33 kΩ    |
| `X13`   | `0x13`  | DNP      | 100 kΩ   |
| `X14`   | `0x14`  | 100 kΩ   | DNP      |
| `X15`   | `0x15`  | 33 kΩ    | DNP      |
| `X16`   | `0x16`  | 11 kΩ    | DNP      |
| `X17`   | `0x17`  | 0 Ω      | DNP      |

## Quick start

```rust
use ti_adc_expander::{Ads7138, Address, OversamplingRatio};

// Create the driver — all channels start as `Unconfigured`
let mut adc = Ads7138::new(i2c, Address::X10);

// Optional: clear the power-on brown-out flag and set oversampling
adc.clear_bor().await?;
adc.set_oversampling(OversamplingRatio::Osr16).await?;

// Configure channels — each call consumes the driver and returns a new one
// with the updated channel type baked into the signature
let mut adc = adc
    .configure_ch0_as_analog().await?
    .configure_ch1_as_analog().await?
    .configure_ch2_as_digital_in().await?
    .configure_ch3_as_digital_out_push_pull().await?;

// Read analog (SCL stretching — available on all chips)
let ch0: u16 = adc.read_ch0().await?;    // 0–4095

// Read without SCL stretching (ADS7x38 / ADS7x28 only)
let ch0: u16 = adc.read_ch0_polled().await?;

// Digital I/O
let high: bool = adc.is_ch2_high().await?;
adc.write_ch3(true).await?;
```

## Analog reads — SCL stretching vs. polled

**SCL-stretch** (`read_chN`, all chips): A raw I²C read causes the device to
stretch SCL while converting (~1.17 µs at 1 MHz). Simple but not supported by
all I²C host controllers.

**Polled** (`read_chN_polled`, ADS7x38 / ADS7x28 only): Enables `STATS_EN`,
triggers conversion via `CNVST`, spins until `OSR_DONE`, then reads from the
`RECENT_CHx` statistics registers. No clock stretching. Not available on
TLA252x, which lacks the statistics register block.

## Alert thresholds (ADS7x38 / ADS7x28)

```rust
// Set 12-bit high/low thresholds for channel 0
adc.set_ch0_thresholds(3000, 500).await?;
adc.set_ch0_hysteresis(4).await?;       // effective = 4 << 3 LSBs
adc.set_ch0_event_count(2).await?;      // alert fires after 3 consecutive violations

// Configure the ALERT pin (open-drain, active-low)
adc.set_alert_channel_mask(0b0000_0001).await?;
adc.configure_alert_pin(false, AlertLogic::ActiveLow).await?;

// Read and clear flags
let highs = adc.event_high_flags().await?;
adc.clear_event_high_flags(highs).await?;
```

## RMS and ZCD (ADS7x28 only)

```rust
use ti_adc_expander::{Ads7128, RmsChannelId, RmsSampleCount};

let mut adc = Ads7128::new(i2c, Address::X10);

// Enable statistics (required for RMS)
adc.enable_dwc().await?;

// Configure and read RMS on channel 0
adc.configure_rms(RmsChannelId::Ain0, true, RmsSampleCount::Samples4096).await?;
let rms: u16 = adc.read_rms().await?;

// Zero-crossing detection
adc.configure_zcd_blanking(false, 10).await?;
adc.set_zcd_gpo_ch0_ch3(
    ZcdGpoValue::Rise1Fall0,
    ZcdGpoValue::Rise0Fall0,
    ZcdGpoValue::Rise0Fall0,
    ZcdGpoValue::Rise0Fall0,
).await?;
adc.set_zcd_gpo_update_mask(0b0000_0001).await?;
```

## Reset

`reset()` issues a soft reset and returns a fresh driver with all channels back
to `Unconfigured`, preserving the chip type:

```rust
let adc = adc.reset().await?;
// adc is now Ads7138<BUS> — all channel type params are Unconfigured
```

## Direct register access

The `device` field exposes the full register map for anything not covered by the
high-level API:

```rust
adc.device.opmode_cfg().modify_async(|r| {
    r.set_conv_mode(ConvMode::Autonomous)
}).await?;

adc.device.sequence_cfg().modify_async(|r| {
    r.set_seq_mode(SeqMode::AutoSequence);
    r.set_seq_start(true);
}).await?;

adc.device.auto_seq_ch_sel().write_with_zero_async(|r| {
    r.set_auto_seq_ch_sel(0x0F)
}).await?;
```

## License

Licensed under either of [Apache License 2.0](LICENSE-APACHE) or
[MIT License](LICENSE-MIT) at your option.
