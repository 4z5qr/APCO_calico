# Time-domain observation and channel occupancy

## Goals
Detect intermittent transmissions without confusing an unobserved frequency with an inactive one. Track both RF energy and validated P25 activity independently.

## Configurable defaults (provisional)
- `fft_integration_ms: 100` — PSD integration interval, not an assumption that a transmission lasts 100 ms.
- `detection_interval_ms: 1000` — cadence for occupancy/state updates.
- `observation_window_s: 600` — rolling statistics window.
- `history_retention_days: 0` — unlimited retention until explicitly configured.
- `on_threshold_db: 9` — excess above local noise estimate.
- `off_threshold_db: 6` — lower threshold for hysteresis.
- `min_on_intervals: 2`, `min_off_intervals: 3` — debouncing, configurable.

## Observation semantics
Each frequency has a time series of intervals:
- `OBSERVED_ACTIVE`: SDR covered this frequency with adequate usable sensitivity and detection threshold exceeded.
- `OBSERVED_INACTIVE`: SDR covered this frequency with adequate sensitivity, no signal detected.
- `UNOBSERVED`: outside tuned usable passband, retuning, dropped samples, receiver fault, or otherwise insufficient coverage.
- `UNKNOWN`: insufficient evidence, e.g. borderline threshold.

Never count `UNOBSERVED` as `OBSERVED_INACTIVE`.

Track two separate axes:
- RF state: present/absent/unknown.
- Protocol state: P25 validated / decoder not locked / not attempted.

An RF signal can remain present while P25 decoding fails; this is not evidence of channel shutdown.

## Occupancy
For a rolling window W:
`occupancy = observed_active_seconds / (observed_active_seconds + observed_inactive_seconds)`.
If denominator is zero, occupancy is undefined (NULL), not 0%.
Also report `coverage = observed_seconds / window_seconds` to avoid misleading statistics.

Track first_seen, last_seen, total_active_seconds, activation_count, longest_burst_seconds, last_valid_p25_frame, and transitions. For dwell-scanning, report activation counts as lower bounds if transitions happen during gaps.

## Event schema
```json
{
  "timestamp_utc": "2026-11-13T20:00:00Z",
  "frequency_hz": 867012500,
  "interval_ms": 1000,
  "coverage": "OBSERVED_ACTIVE",
  "rf_excess_db": 12.4,
  "p25_validated": false,
  "receiver": "lime-rx0"
}
```

## Storage proposal
- `observation_intervals`: start/end, frequency, receiver, RF state, P25 state, PSD metrics, quality flags.
- `channel_events`: transitions, bursts, P25 CC switching, decode successes.
- `channels`: latest known role, site identifiers, first/last seen.

Use UTC timestamps and monotonic clock for interval durations. Merge adjacent identical intervals to limit storage growth. Retain aggregates per minute/hour for longer-term visualization.

## Implementation plan
1. Timestamped PSD snapshots and explicit receiver coverage windows.
2. Threshold hysteresis and debounced per-frequency activity tracker.
3. SQLite intervals and rolling occupancy statistics.
4. Live P25 decoder state and site-role association.
5. Multi-receiver coverage fusion and duty-cycle confidence estimates.

## Caveat
A 1-second detection cadence cannot reliably count bursts shorter than 1 second. For such signals, derive occupancy from shorter PSD frames (e.g. 50–100 ms) and aggregate into 1-second summaries. Calibration, fading and adjacent-channel leakage affect false alarms.
