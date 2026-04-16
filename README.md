# ads7138

An async Rust driver for the [Texas Instruments ADS7138](https://www.ti.com/product/ADS7138) —
a 12-bit, 8-channel, I2C ADC with configurable GPIO, digital window comparator, alert output,
oversampling, and RMS calculation.

Built on [`embedded-hal-async`](https://crates.io/crates/embedded-hal-async) and
[`device-driver`](https://crates.io/crates/device-driver). All methods return the I2C bus
error type directly — no wrapper enum.

## Features

- Async-only, `no_std` compatible
- Typestate-enforced channel configuration — analog/digital-in/digital-out modes are checked at
  compile time; calling `read_ch0()` on a channel configured as digital output is a type error
- Two analog read strategies:
  - SCL-stretch read (one I2C transaction, uses clock stretching)
  - Polled read (no clock stretching; uses `CNVST` + `OSR_DONE` polling)
- Full register access via the `device` field for anything not covered by the high-level API
- Optional `defmt` support via the `defmt` feature

## Cargo.toml

```toml
[dependencies]
ads7138 = "0.1"
```

To enable `defmt`:

```toml
ads7138 = { version = "0.1", features = ["defmt"] }
```

## I2C address

The device address is configured by resistors on the `ADDR` pin. Use the `Address` enum:

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
use ads7138::{Ads7138, Address, OversamplingRatio};

// Create the driver (all channels start as `Unconfigured`)
let mut adc = Ads7138::new(i2c, Address::X10);

// Optional: clear the power-on brown-out flag and set oversampling
adc.clear_bor().await?;
adc.set_oversampling(OversamplingRatio::Osr16).await?;

// Configure channels — each configure call consumes the driver and returns
// a new one with the updated channel type in the signature
let mut adc = adc
    .configure_ch0_as_analog().await?
    .configure_ch1_as_analog().await?
    .configure_ch2_as_digital_in().await?
    .configure_ch3_as_digital_out_push_pull().await?;

// Read analog channels (SCL stretching)
let ch0: u16 = adc.read_ch0().await?;          // 0–4095
let ch1: u16 = adc.read_ch1().await?;

// Read without SCL stretching (polls OSR_DONE)
let ch0: u16 = adc.read_ch0_polled().await?;

// Read/write digital channels
let high: bool = adc.is_ch2_high().await?;
adc.write_ch3(true).await?;
```

## Analog reads — SCL stretching vs. polled

The ADS7138 supports two conversion trigger modes:

**SCL-stretch** (`read_chN`): A raw I2C read causes the device to stretch SCL while it converts
(~1.17 µs at 1 MHz). This is the simplest approach but some I2C host controllers do not support
clock stretching.

**Polled** (`read_chN_polled`): Enables the statistics module (`STATS_EN`), triggers the
conversion via `CNVST`, spins until `OSR_DONE` is set, then reads the result from the
`RECENT_CHx` registers. No SCL stretching required.

## Alert thresholds (analog channels)

```rust
// Set 12-bit high/low thresholds and a 4-bit hysteresis for channel 0
adc.set_ch0_thresholds(3000, 500).await?;   // high=3000, low=500 (out-of-window alert)
adc.set_ch0_hysteresis(4).await?;           // effective hysteresis = 4 << 3 LSBs
adc.set_ch0_event_count(2).await?;          // alert fires after 3 consecutive violations

// Enable alert for channel 0 and configure the ALERT pin
adc.set_alert_channel_mask(0b0000_0001).await?;
adc.configure_alert_pin(false, AlertLogic::ActiveLow).await?;   // open-drain, active-low

// Later, read and clear flags
let highs = adc.event_high_flags().await?;
adc.clear_event_high_flags(highs).await?;
```

## Reset

`reset()` writes the `RST` bit and returns a fresh driver with all channels back to `Unconfigured`:

```rust
let adc = adc.reset().await?;
// adc is now Ads7138<BUS> — all channel type params are Unconfigured
```

## Direct register access

The `device` field exposes the full generated register map for anything not covered by the
high-level API:

```rust
// Enable autonomous conversion mode
adc.device.opmode_cfg().modify_async(|r| {
    r.set_conv_mode(ConvMode::Autonomous)
}).await?;

// Configure auto-sequence over channels 0–3
adc.device.sequence_cfg().modify_async(|r| {
    r.set_seq_mode(SeqMode::AutoSequence);
    r.set_seq_start(true);
}).await?;
adc.device.auto_seq_ch_sel().write_with_zero_async(|r| {
    r.set_auto_seq_ch_sel(0x0F)
}).await?;
```

## License

Licensed under either of [Apache License 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at
your option.
