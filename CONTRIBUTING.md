# Contributing

Thank you for contributing to KickerBoard.

## Scope

Contributions must relate directly to the LT3751 kicker system. Do not add A3930, Allegro motor drivers, BLDC commutation, Hall sensors, or unrelated robot boards.

## Before submitting

- identify the affected board and firmware revision;
- explain the motivation and expected effect;
- include calculations for voltage, current, energy, thermal, or timing changes;
- distinguish simulation from bench measurement;
- attach raw data when claiming experimental improvement;
- update documentation and changelog;
- avoid committing proprietary or unlicensed third-party files.

## Hardware changes

Hardware pull requests should include:

- schematic diff or marked-up PDF;
- affected BOM entries and manufacturer part numbers;
- rating and tolerance review;
- creepage/clearance impact;
- fabrication and assembly impact;
- validation plan;
- photographs or captures when hardware has been tested.

## Firmware changes

Firmware pull requests should include:

- target platform and toolchain version;
- safe default output behavior;
- state-machine impact;
- timeout and fault behavior;
- serial-interface changes;
- test procedure and result.

## Measurement submissions

State the instrument, probe, bandwidth, sample rate, test conditions, waveform window, and calculation method. Preserve raw files whenever possible.

## Commit style

Use concise conventional-style messages, for example:

```text
docs: explain energy-percentage mapping
firmware: add charge timeout state
hardware: update feedback divider review
sim: document discharge RMS window
```

## Safety review

Changes that can increase stored energy, voltage, current, trigger duration, or accessible high-voltage exposure require explicit safety review before merge.
