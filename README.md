# KickerBoard

Open-source high-voltage kicker module for RoboCup Small Size League robots, developed by **TauraBots** and based on the **Analog Devices LT3751** capacitor charger controller.

> [!WARNING]
> This project operates with hazardous high voltage and stores significant energy in a capacitor bank. Even after power is removed, the capacitor may remain charged. Work only with appropriate supervision, insulated tools, discharge procedures, shielding, interlocks, and measurement equipment rated for the voltage and pulse energy involved.

## Project scope

This repository is exclusively dedicated to the LT3751-based kicker system: charging stage, high-voltage capacitor bank, trigger stage, control firmware, PCB, simulations, measurements, assembly, testing, and scientific documentation.

Motor drivers, BLDC electronics, Allegro A3930 circuits, Hall commutation, and unrelated TauraBots boards are intentionally outside this repository.

## Current reference configuration

| Parameter | Reference value |
|---|---:|
| Input supply used in the documented reference | 12.6 V |
| Nominal capacitor voltage | up to 200 V |
| Energy-storage capacitor | 1000 µF |
| Stored energy at 200 V | 20 J |
| Simulated charge time to approximately 200 V | approximately 445–450 ms |
| Simulated average input current during charging | approximately 5.20 A |
| Simulated discharge RMS current | approximately 8.34 A |
| Simulated discharge peak current | approximately 24.36 A |
| Main charging spectral component | approximately 60 kHz |
| Main controller | LT3751 |
| Control platform used during development | ESP32-C3 |

The values above are design-reference and simulation results unless a document explicitly identifies them as bench measurements. See [Measurements](docs/measurements.md) and [Validation status](docs/validation-status.md).

The capacitor energy is:

```text
E = 1/2 × C × V²
E = 1/2 × 0.001 × 200² = 20 J
```

## Features

- LT3751-based flyback capacitor charger
- adjustable high-voltage setpoint
- charge-enable control from an external microcontroller
- DONE and FAULT monitoring
- voltage feedback for supervisory control
- independent kick trigger output stage
- ESP32-C3 control example
- serial commands for charging, stopping, status, setpoint adjustment, and firing
- simulation workflow for charge time, input current, discharge current, RMS calculation, and FFT
- manufacturing-oriented organization for schematic, PCB, Gerbers, BOM, and assembly data

## Repository structure

```text
KickerBoard/
├── README.md
├── CHANGELOG.md
├── CITATION.cff
├── CONTRIBUTING.md
├── LICENSE
├── SECURITY.md
├── docs/
│   ├── overview.md
│   ├── theory-of-operation.md
│   ├── measurements.md
│   ├── validation-status.md
│   ├── safety.md
│   ├── firmware.md
│   ├── manufacturing.md
│   └── repository-map.md
├── firmware/
│   └── README.md
├── hardware/
│   ├── README.md
│   ├── easyeda/
│   ├── gerbers/
│   ├── schematic/
│   ├── bom/
│   └── pick-and-place/
├── simulations/
│   └── README.md
├── measurements/
│   └── README.md
├── media/
│   └── README.md
└── paper/
    └── README.md
```

Binary design files, photographs, scope captures, Gerbers, EasyEDA source, LTspice projects, and the IEEE paper are added only when their exact source files have been verified. This prevents placeholder material from being mistaken for production-ready data.

## System overview

The board converts the robot battery voltage into a regulated high-voltage DC bus. The LT3751 controls a flyback converter and terminates charging when the feedback network indicates that the selected capacitor voltage has been reached. A microcontroller supervises charging through digital control and status lines. The stored energy is then delivered to the solenoid through a separate high-current trigger stage.

Typical signal names used in the ESP32-C3 test firmware:

| ESP32-C3 GPIO | Signal |
|---:|---|
| GPIO0 | READ, capacitor-voltage feedback |
| GPIO1 | KICK, trigger control |
| GPIO2 | FAULT# |
| GPIO3 | DONE# |
| GPIO4 | CHARGE |

Verify the board revision before using this mapping. Pin assignments are not a substitute for the released schematic.

## Firmware behavior

The development firmware provides serial commands to:

- start charging;
- stop charging;
- trigger the kicker;
- show full status;
- change the voltage setpoint;
- convert a requested kick percentage into a voltage target;
- optionally fire automatically after the requested percentage is reached.

Automatic firing must include timeout, FAULT handling, valid-voltage checks, minimum trigger-pulse spacing, and a safe discharge procedure.

## Manufacturing files

The intended release set includes:

- EasyEDA source project;
- schematic PDF;
- PCB source;
- Gerber archive;
- drill files;
- bill of materials;
- pick-and-place file;
- assembly drawings;
- fabrication notes;
- board revision history.

Do not manufacture from screenshots or from documentation tables alone. Use a tagged release containing the verified fabrication package.

## Simulation and measurements

The project documentation distinguishes among:

1. analytical calculations;
2. LTspice simulation results;
3. oscilloscope or multimeter bench measurements;
4. inferred values calculated from captured waveforms.

This distinction is important because a simulated 24.36 A discharge peak, for example, is not automatically an experimentally validated peak current.

## Sponsors and acknowledgements

PCB design was developed using [EasyEDA](https://easyeda.com/), and PCB fabrication and assembly support was provided by [JLCPCB](https://jlcpcb.com/).

The project also acknowledges technical support, samples, components, or reference material from:

- [Analog Devices](https://www.analog.com/) — LT3751 controller and technical documentation;
- [Coilcraft](https://www.coilcraft.com/) — magnetic components and design support;
- [Littelfuse](https://www.littelfuse.com/) — protection and power-semiconductor components.

Acknowledgement does not imply that every listed company has certified or endorsed the complete board.

## Documentation and paper

The associated Portuguese IEEE-style manuscript is titled:

**Projeto, Simulação, Implementação e Validação Experimental de um Módulo Kicker para Robôs de Futebol SSL Baseado no LT3751**

The paper directory will contain the verified LaTeX source, bibliography, figures, and compiled PDF used for publication.

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request. Contributions involving safety-critical changes must include calculations, component ratings, board revision, and validation evidence.

## License

Documentation and software licensing are described in [LICENSE](LICENSE). Hardware source releases should explicitly identify the applicable open-hardware license in each tagged release.

## Citation

Use [CITATION.cff](CITATION.cff) when citing this repository.

## Team

Developed by **TauraBots**, Universidade Federal de Santa Maria (UFSM), for research and development in RoboCup Small Size League robotics.
