# Measurements and simulation summary

This document records known project values and classifies their evidence level.

## Current dataset

| Quantity | Value | Evidence class | Status |
|---|---:|---|---|
| Capacitor | 1000 µF | design parameter | reference configuration |
| Maximum target voltage | 200 V | design parameter | firmware/simulation reference |
| Stored energy at 200 V | 20 J | analytical calculation | verified by formula |
| Charge time to approximately 200 V | 445–450 ms | LTspice simulation | awaiting bench comparison |
| Average input current during charging | 5.20 A | LTspice waveform analysis | awaiting bench comparison |
| Discharge RMS current | 8.34 A | simulated waveform calculation | awaiting shunt/probe validation |
| Discharge peak current | 24.36 A | LTspice simulation | awaiting shunt/probe validation |
| Dominant charging component | approximately 60 kHz | FFT of simulated current | awaiting measured FFT |

## RMS calculation

For sampled current values `i[k]`:

```text
Irms = sqrt((1/N) × sum(i[k]²))
```

The integration window must be stated. A pulse RMS calculated only during the discharge interval differs from RMS over an entire robot duty cycle.

## FFT procedure

For reproducible spectral analysis:

1. export a uniformly sampled current waveform;
2. remove DC offset when appropriate;
3. document sample rate and time window;
4. apply and identify the selected window function;
5. calculate a one-sided amplitude spectrum;
6. state whether amplitude is peak, RMS, or power spectral density;
7. retain the original waveform with the processed result.

## Required bench records

Each experimental capture should include:

- board revision and serial identifier;
- battery voltage and current limit;
- capacitor part number and measured capacitance;
- programmed voltage target;
- probe model, attenuation, bandwidth, and grounding method;
- shunt or current-probe calibration;
- ambient conditions;
- waveform file and screenshot;
- calculation script version;
- uncertainty or expected measurement error.

## Naming convention

Recommended format:

```text
YYYY-MM-DD_revX_test-name_condition_instrument.ext
```

Example:

```text
2026-07-25_revA_charge-200V_12V-input_scope.csv
```
