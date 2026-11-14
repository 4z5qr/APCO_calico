# APCO_calico — P25 wideband spectrum scanner v0.1

First milestone: LimeSDR IQ → averaged FFT → candidate signal regions.

**Not yet a P25 detector or a full frequency sweep.** This prototype captures one 5 MHz window.

## Ubuntu dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libsoapysdr-dev libfftw3-dev soapysdr-tools
SoapySDRUtil --find
```

The LimeSDR Soapy module must also be installed and visible in `SoapySDRUtil --find`.

## Build and run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/p25scanner 867.5 5 'driver=lime'
```

Arguments: center MHz, sample rate MS/s, SoapySDR device args.

Produces `spectrum.csv` (all FFT bins) and `candidates.csv` (regions above a local noise floor). `relative_db` and `snr_proxy_db` are **not calibrated dBm or receiver SNR**.

## Caveats / next milestone

- No protocol classification, no persistent detection and no sweep over 865–870 MHz yet.
- Gain, antenna and sample rate need to be checked on actual hardware.
- A single 5 MHz capture may lose usable spectrum at the edges; sweep with overlap.
- The center DC artifact is masked by 8 bins.
- Candidate extraction may split a modulated P25 channel into multiple regions. Next: channel-width-aware grouping, occupancy persistence, P25 sync validation.

## Experimental Qt GUI

Install Qt development dependencies:

```bash
sudo apt install -y qtbase5-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/p25scanner_gui
```

The GUI provides start/stop frequencies, tuning step, sample rate, gain,
dwell time, total scan time, IQ capture duration, and a trigger menu.

**Current limitations:**
- SNR-proxy peak threshold triggers raw wideband CF32 IQ recordings.
- C4FM and CQPSK trigger choices are placeholders and intentionally do not
  record until channelization, symbol recovery and a classifier exist.
- The overview currently shows the most recent tuning window, not yet a
  persisted accumulated full-band history.
- The waterfall updates once per dwell, not continuously at FFT frame rate.
- The implementation is experimental and has not yet been validated with
  a physical LimeSDR-USB.
