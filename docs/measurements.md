# Measurements and simulation summary

This document records the latest consolidated dataset and classifies each value by evidence type.

## Consolidated simulation setup

- Input source: 12.6 V
- Lumped source/cable resistance: 50 mΩ
- Storage capacitor: 1000 µF
- Target capacitor voltage: approximately 200 V
- Transient duration: 600 ms
- Maximum simulation step: 1 µs
- Input-current window: 0–450 ms
- Rectifier-current window: 200–220 ms
- Solenoid-current window: 450–500 ms

## Consolidated results

| Quantity | Value | Evidence |
|---|---:|---|
| Capacitor final voltage | approximately 200 V | LTspice simulation |
| Charging time | approximately 445 ms | LTspice simulation |
| Stored energy at 200 V | approximately 20 J | analytical calculation |
| Input current, average | 5.045 A | simulation, 0–450 ms |
| Input current, RMS | 5.815 A | simulation, 0–450 ms |
| Input current, narrow peak | 116.103 A | simulation, 0–450 ms |
| D1 current, average | 0.380 A | simulation, 200–220 ms |
| D1 current, RMS | 0.659 A | simulation, 200–220 ms |
| D1 current, peak | 1.741 A | simulation, 200–220 ms |
| Solenoid current, RMS | 5.071 A | simulation, 450–500 ms |
| Solenoid current, peak | 21.705 A | simulation, 450–500 ms |
| Dominant recharge component | approximately 60 kHz | FFT of simulated D1 current |
| Estimated energy efficiency | approximately 69.9% | derived from simulated averages |

The 116.103 A input maximum is an extremely narrow simulated startup/switching transient. It must not be interpreted as a continuous battery-current requirement. Its magnitude is highly sensitive to ESR, cable inductance, source model, initial conditions, and numerical settings.

Earlier preliminary documents reported 5.20 A average input current, 8.34 A solenoid RMS current, and 24.36 A solenoid peak current. Those values came from an earlier model and are superseded by the consolidated 12.6 V, 50 mΩ-source simulation above.

## LCR measurements of the solenoid

These are experimental small-signal measurements.

| Test frequency | Series inductance Ls | Series resistance Rs | Q |
|---:|---:|---:|---:|
| 100 Hz | 12.340 mH | 5.65 Ω | 1.362 |
| 120 Hz | 11.917 mH | 6.22 Ω | 1.450 |
| 1 kHz | 6.957 mH | 26.61 Ω | 1.666 |
| 10 kHz | 2.996 mH | 96.45 Ω | 1.961 |
| 100 kHz | 2.043 mH | 442.70 Ω | 2.900 |

The primary transient model uses the 100 Hz point, `L = 12.34 mH` and `R = 5.65 Ω`, as a grounded low-frequency approximation. This is not a full nonlinear electromechanical model: plunger position, magnetic saturation, temperature, and motion can change the instantaneous parameters during a real kick.

## Functional hardware validation

The prototype has been functionally observed completing this sequence:

1. auxiliary 5 V supply operation;
2. flyback enable;
3. capacitor charging;
4. charge-complete indication;
5. SCR triggering;
6. mechanical solenoid actuation;
7. recharge after the shot.

This is functional validation, not quantitative oscilloscope validation. Quantitative current, efficiency, EMI, and repeatability comparisons remain future work.

## RMS calculation

For sampled current values `i[k]`:

```text
Irms = sqrt((1/N) × sum(i[k]²))
```

The integration window must always be stated.

## FFT interpretation

The D1 recharge current is repetitive and shows a dominant component near 60 kHz with harmonics in the selected simulation window. The solenoid current is a single damped pulse, so its FFT describes spectral-energy distribution rather than a strict periodic fundamental.

## Required bench records

Each quantitative experimental capture should include board revision, input conditions, capacitor identification, voltage target, instrument and probe data, bandwidth, sample rate, raw waveform, analysis window, calibration, and uncertainty.
