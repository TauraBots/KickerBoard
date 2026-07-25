# System overview

KickerBoard is a high-voltage energy-storage and pulse-delivery module for RoboCup Small Size League robots.

## Functional blocks

1. **Low-voltage input** — receives energy from the robot battery.
2. **Flyback charging converter** — the LT3751 controls the primary switching stage and transformer energy transfer.
3. **High-voltage rectification** — the transformer secondary charges the storage capacitor through a suitable high-voltage diode network.
4. **Feedback and charge termination** — the output-voltage divider reports the capacitor voltage and determines the final charging setpoint.
5. **Supervisory interface** — CHARGE, DONE#, FAULT#, READ, and KICK connect the board to an external controller.
6. **Energy-storage capacitor** — stores the energy delivered to the solenoid.
7. **Trigger stage** — a pulse-rated semiconductor discharges the capacitor through the kicker coil.
8. **Protection and discharge provisions** — include current limiting, voltage-rated clearances, safe discharge, and fault handling.

## Reference energy

For a 1000 µF capacitor:

| Voltage | Stored energy |
|---:|---:|
| 50 V | 1.25 J |
| 100 V | 5 J |
| 150 V | 11.25 J |
| 180 V | 16.2 J |
| 200 V | 20 J |

The energy increases with the square of voltage. A 10% voltage increase produces approximately a 21% energy increase.

## Interface reference

The development ESP32-C3 mapping is:

| GPIO | Function | Direction at ESP32-C3 |
|---:|---|---|
| GPIO0 | READ | analog input |
| GPIO1 | KICK | digital output |
| GPIO2 | FAULT# | digital input |
| GPIO3 | DONE# | digital input |
| GPIO4 | CHARGE | digital output |

Active-low status signals must use the electrical levels and pull resistors defined by the released schematic.

## Scope boundary

This project does not contain motor-drive electronics. No A3930, Allegro BLDC driver, phase bridge, Hall sequence, or motor-control firmware belongs in this repository.
