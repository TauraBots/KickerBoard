# Theory of operation

## Charging cycle

When CHARGE is asserted, the LT3751 drives the converter switching stage. Current rises in the transformer primary, magnetic energy is transferred to the secondary, and the rectifier charges the high-voltage capacitor. The controller monitors its feedback network and stops or regulates charging near the configured voltage.

The firmware must stop charging immediately when a fault is detected, a timeout expires, the measured voltage becomes implausible, or the user requests a stop.

## Stored energy

The capacitor stores:

```text
E = 1/2 × C × V²
```

For `C = 1000 µF` and `V = 200 V`, `E = 20 J`.

Because energy is quadratic with voltage, kick percentage should not normally be mapped linearly to voltage when the desired variable is energy. For an energy command `p` from 0 to 100%:

```text
Vtarget = Vmax × sqrt(p / 100)
```

Examples for a 200 V maximum:

| Energy command | Voltage target |
|---:|---:|
| 25% | 100 V |
| 50% | 141.4 V |
| 75% | 173.2 V |
| 100% | 200 V |

A linear voltage mapping may still be exposed as a separate test mode, but it must not be labeled linear energy.

## Trigger cycle

After the voltage target is reached and DONE#/READ indicate a valid state, the charging converter is disabled. A controlled pulse is then applied to KICK. The pulse-rated switch connects the charged capacitor to the solenoid, creating a high-current transient.

The trigger routine must enforce:

- valid voltage range;
- no active FAULT#;
- CHARGE disabled before firing;
- bounded pulse width;
- minimum interval between shots;
- post-shot voltage verification;
- lockout after timeout or inconsistent status.

## Measurement model

A scaled analog feedback signal can estimate capacitor voltage:

```text
Vcap = ADC_voltage × divider_ratio × calibration_factor + offset
```

Divider tolerance, ADC reference error, loading, noise, and board leakage must be calibrated experimentally. Firmware constants are board-revision-specific.
