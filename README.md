# RP2040_ADC_Calibrator

The RP2040 ADC has documented nonlinearities due to a production issue.  This pico-arduino sketch can be used to self-calibrate an ADC to help to linearise the readings.  

The firmware creates a DC signal using delta-sigma modulation and an external filter.  This signal scans the ADC, and the scan is used to create a table of corrections that is saved to flash.  An application can load this table, and use it to correct the nonlinearities of the ADC.

The RP2040 has a single ADC that is multiplexed through the four ADC pins, so a calibration from any ADC input will work with all the other inputs.

## Hardware

<img width="1145" height="809" alt="image" src="https://github.com/user-attachments/assets/4717a6d3-4b4f-4be8-b74e-e047ebc6e706" />

An analog filter is needed to convert the delta-sigma bitstream into a continuous voltage.  The diagram above uses a sallen-key filter, with a TL07x op amp.  You can also use a single-rail op-amp like the MCP600x, in which case you don't need the clamping diodes and resistor after the op-amp output.

## Operation

The firmware is operated using the serial terminal.

### ADC Calibration Commands

| Command | Description |
|---------|-------------|
| `c` | Run full calibration (histogram sweep, build LUT, save to flash) |
| `t` | Quick DAC/ADC test (5 points: 0, 1024, 2048, 3072, 4095) |
| `v` | Verify calibration (compare raw vs corrected at 7 test points) |
| `d` | Dump CSV to serial (code, histogram count, correction for all 4096 codes) |
| `h` | Show help |

## Firmware

### Calibration Process

| Step | Function | Purpose |
|------|----------|---------|
| 1 | `collect_histogram()` | Sweep DAC 0-4095, record which ADC codes appear |
| 2 | `build_correction_lut()` | Calculate correction for each code from histogram |
| 4 | `smooth_correction_table()` | Remove discontinuities at bit-9 boundaries (512, 1536, 2560, 3584) |
| 5 | `save_calibration()` | Write to LittleFS (adc_cal.bin + adc_cal.csv) |

### Key Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `SAMPLES_PER_LEVEL` | 128 | Samples collected at each DAC level |
| `NUM_SWEEPS` | 4 | Up/down sweeps (averages hysteresis) |
| `SETTLE_US` | 200 | Settling time after DAC change (µs) |
| `PIO clkdiv` | 256 | Sigma-delta bit rate: 488 kHz |

## Application Code

```ADCProfile.hpp``` provides code to load the corrections table.
