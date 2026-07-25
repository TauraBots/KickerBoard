# Simulations

This directory is reserved for the verified LTspice project, component models whose redistribution terms permit inclusion, raw waveform exports, FFT data, and reproducible analysis notes.

## Consolidated model

The current documented model uses:

- 12.6 V source;
- 50 mΩ lumped source resistance;
- 1000 µF storage capacitor;
- approximately 200 V target;
- 600 ms transient analysis;
- 1 µs maximum step;
- measured solenoid approximation of 12.34 mH and 5.65 Ω at 100 Hz.

See `docs/measurements.md` for the consolidated numerical results.

## Planned contents

```text
simulations/
├── ltspice/
├── models/
├── raw/
├── exports/
├── fft/
└── scripts/
```

Do not commit a third-party SPICE macromodel unless its redistribution license permits it. A download note and source citation may be used instead.
