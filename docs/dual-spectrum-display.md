# Dual spectrum display: accumulated overview + live waterfall

## Layout

**Overview (upper pane):** full configured scan range, updated after each completed dwell/scan pass. Render last-observed PSD and selectable historical aggregates (peak, mean, occupancy). Mark the current receiver passband with a rectangle. Do not treat unsampled frequencies as zero-power or idle.

**Live waterfall (lower pane):** only the currently tuned usable receiver passband, high time resolution, newest rows at top. Frequency coordinates must be absolute and aligned with the overview. On retune, reset or visually segment the waterfall; never stitch unrelated center frequencies into one continuous frequency axis.

## Data pipeline

SoapySDR IQ -> windowed FFT -> per-frame PSD -> frame timestamps
  -> ring buffer for live waterfall
  -> frequency/time-binned aggregates for overview
  -> persistent observation intervals and channel events

For each overview frequency bin, maintain:
- last observed power + observation timestamp
- max/mean power over selected observation window
- occupancy and observation coverage fractions
- current state: observed / stale / never observed

Unobserved gaps are rendered distinctly (gray/hatched), not as quiet spectrum. Older data should have a visible age indicator or fade. Mark actual usable passband, not nominal sample-rate edges; overlap sweep windows.

## Suggested initial settings
- overview range: user configurable, e.g. 850–870 MHz
- instantaneous receiver span: 5 MHz
- waterfall update: 10–20 rows/s, with per-row mean and max PSD options
- history window: 10 min, adjustable
- overview bin size: configurable (e.g. 2.5 kHz)
- timestamps: UTC for storage, monotonic clock for durations

## Interaction (later)
Click an overview frequency to tune, inspect occupancy timeline, display discriminator levels / CQPSK constellation, and optionally validate P25 synchronization. Preserve history while retuning.

## Implementation note
The existing v0.1 C++ scanner only outputs one averaged spectrum. This document describes the proposed UI and buffering model; no GUI or sweep implementation is included yet.
