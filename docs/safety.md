# Safety

KickerBoard contains a high-voltage capacitor capable of storing approximately 20 J at the reference 200 V and 1000 µF configuration. This energy can cause severe injury, burns, involuntary movement, damaged instruments, fire, and destructive component failure.

## Minimum controls

- Work under qualified adult or laboratory supervision.
- Use a current-limited and fused input supply.
- Install a physical emergency disconnect.
- Enclose high-voltage conductors and the capacitor.
- Use insulated tools and voltage-rated probes.
- Keep one hand away from the assembly when probing energized hardware.
- Never rely on firmware as the only safety mechanism.
- Verify zero voltage with an appropriate instrument before touching the circuit.
- Use a permanently connected or positively controlled bleeder/discharge path sized for voltage, pulse energy, and temperature.
- Label the board with maximum voltage and stored-energy warnings.
- Keep the solenoid mechanically restrained during testing.

## Discharge verification

After disabling the input:

1. prevent any new CHARGE command;
2. apply the designed discharge path;
3. wait the documented discharge interval;
4. measure capacitor voltage using a rated probe;
5. confirm the value is below the laboratory's safe-touch threshold;
6. repeat the measurement before handling.

Do not short the capacitor directly with a screwdriver or loose wire.

## Measurement hazards

Ordinary oscilloscope ground clips may be earth referenced. Incorrect connection can short a switching node or high-voltage rail. Use an appropriate differential probe, isolated measurement setup, or a properly designed low-side shunt measurement whose reference is understood.

## Firmware safeguards

Firmware should default outputs to the non-charging, non-firing state during reset and boot. It should reject firing when status is invalid, limit pulse duration, include a charging timeout, and require a new explicit command after faults.

## Release requirement

A production-ready release must include creepage and clearance review, voltage and pulse-current ratings, thermal checks, fuse specification, safe-discharge timing, and a documented pre-power checklist.
