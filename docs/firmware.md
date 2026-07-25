# Firmware specification

The firmware supervises charging and firing but must not be treated as the sole safety layer.

## Development GPIO mapping

| GPIO | Signal |
|---:|---|
| 0 | READ |
| 1 | KICK |
| 2 | FAULT# |
| 3 | DONE# |
| 4 | CHARGE |

Confirm the mapping against the released schematic and board revision before use.

## Command model

The development serial interface supports these command classes:

- start charging;
- stop charging;
- request status;
- select a voltage or energy percentage;
- request a controlled kick;
- abort and enter a safe state.

## Percentage interpretation

The recommended public command represents stored-energy percentage:

```text
Vtarget = Vmax × sqrt(percent / 100)
```

The interface should report both requested percentage and calculated voltage. Values outside the configured range must be rejected.

## State machine

Recommended states:

```text
SAFE → IDLE → CHARGING → READY → FIRING → RECOVERY → IDLE
                    ↘ FAULT ← any active state
```

### SAFE

Outputs remain inactive while hardware, ADC, and status inputs initialize.

### IDLE

The board is not charging. Commands and status may be received.

### CHARGING

The controller enables charging and monitors voltage, DONE#, FAULT#, elapsed time, and plausibility.

### READY

The requested target has been reached within tolerance. Charging is disabled before any firing transition.

### FIRING

A hardware-bounded trigger request is issued. Exact timing parameters belong in the verified firmware release and test report rather than general documentation.

### RECOVERY

The firmware waits for status stabilization, checks residual voltage, and enforces shot spacing.

### FAULT

Charging and firing are disabled. Recovery requires an explicit reset or acknowledgement after the fault source has cleared.

## Automatic-fire sequence

Automatic firing after reaching a requested percentage must require all of the following:

- valid percentage and target calculation;
- no active fault;
- charge completion confirmed by independent indicators;
- voltage within a configured tolerance band;
- charging disabled before trigger;
- no timeout or sensor disagreement;
- explicit arming for that single shot.

The automatic-fire option should be disabled by default after reset.

## Telemetry

Recommended status output:

```text
state, requested_percent, target_voltage, measured_voltage,
done, fault, charge_enabled, armed, timeout_status
```

Calibration constants and safety limits must be tied to a board and firmware revision.
