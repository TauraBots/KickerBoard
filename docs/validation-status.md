# Validation status

## Status definitions

- **Calculated** — obtained directly from an equation and stated assumptions.
- **Simulated** — produced by an electrical simulation model.
- **Bench observed** — observed in hardware but not yet reproduced with a complete measurement record.
- **Experimentally validated** — reproduced with identified hardware, instruments, raw data, and procedure.
- **Release verified** — reviewed and included in a tagged manufacturing release.

## Current matrix

| Item | Current status | Required next evidence |
|---|---|---|
| 20 J at 1000 µF and 200 V | Calculated | capacitance and voltage tolerance analysis |
| approximately 450 ms charge time | Simulated | oscilloscope capture of Vcap and CHARGE |
| 5.20 A average input current | Simulated | calibrated input-current waveform |
| 24.36 A discharge peak | Simulated | pulse-rated shunt or current probe capture |
| 8.34 A discharge RMS | Simulated | raw current CSV and declared RMS window |
| approximately 60 kHz dominant component | Simulated | measured FFT with sampling metadata |
| ESP32-C3 GPIO mapping | Development reference | released schematic and firmware tag |
| Automatic fire after percentage target | Specification | source implementation and hardware test |
| Schematic | Pending source upload | reviewed PDF and source project |
| PCB/Gerbers/BOM | Pending source upload | tagged fabrication package |

No item marked simulated should be presented as experimentally validated in papers, release notes, or promotional material.
