# KickerBoard

Open-source high-voltage kicker module for RoboCup Small Size League robots, developed by **TauraBots** and based on the **Analog Devices LT3751** capacitor-charger controller.

> [!WARNING]
> This project operates with hazardous high voltage and stores significant energy. Even after power is removed, the capacitor may remain charged. Work only in an appropriate supervised laboratory with shielding, interlocks, rated instruments, and a verified discharge procedure.

## Scope

This repository is exclusively dedicated to the LT3751 kicker system: flyback charging, high-voltage storage, SCR trigger stage, ESP32-C3 supervision, PCB, simulations, LCR characterization, functional testing, manufacturing files, and scientific documentation.

Motor drivers, BLDC electronics, Hall commutation, Allegro A3930 circuits, and unrelated TauraBots boards are intentionally excluded.

## Acknowledgements

The project acknowledges support, parts, tools, or technical material from [JLCPCB](https://jlcpcb.com/), and [EasyEDA](https://easyeda.com/).


## Reference configuration

| Parameter | Consolidated value |
|---|---:|
| Input range considered | 12–12.6 V |
| Simulation source | 12.6 V with 50 mΩ series resistance |
| Capacitor | 1000 µF |
| Target voltage | approximately 200 V |
| Stored energy | approximately 20 J |
| Simulated charging time | approximately 445 ms |
| Simulated input current, average | 5.045 A |
| Simulated input current, RMS | 5.815 A |
| Simulated input narrow peak | 116.103 A |
| Simulated D1 current, average/RMS/peak | 0.380 / 0.659 / 1.741 A |
| Simulated solenoid current, RMS | 5.071 A |
| Simulated solenoid current, peak | 21.705 A |
| Dominant recharge spectral component | approximately 60 kHz |
| Estimated simulated energy efficiency | approximately 69.9% |
| Controller | LT3751 |
| Flyback transformer | Coilcraft GA3459-BL |
| Main switching MOSFET in documented prototype | IRF640 |
| High-voltage rectifier | ES3J |
| Trigger device | TYN640-class SCR |
| Auxiliary regulator | AP62300 |
| Development controller | ESP32-C3 |
| Approximate PCB dimensions | 80 mm × 50 mm |

The numerical current and efficiency values above are simulation-derived. Experimental evidence currently consists of LCR measurements and functional prototype validation. See [Measurements](docs/measurements.md) and [Validation status](docs/validation-status.md).

## Solenoid characterization

The actuator was measured using the instrument's series-equivalent model:

| Frequency | Ls | Rs | Q |
|---:|---:|---:|---:|
| 100 Hz | 12.340 mH | 5.65 Ω | 1.362 |
| 120 Hz | 11.917 mH | 6.22 Ω | 1.450 |
| 1 kHz | 6.957 mH | 26.61 Ω | 1.666 |
| 10 kHz | 2.996 mH | 96.45 Ω | 1.961 |
| 100 kHz | 2.043 mH | 442.70 Ω | 2.900 |

The primary LTspice discharge model uses the 100 Hz point as a low-frequency approximation. It does not model magnetic saturation, plunger motion, temperature rise, or full electromechanical coupling.

## Energy command

For a user command representing stored-energy percentage:

```text
Vtarget = Vmax × sqrt(percent / 100)
```

At `Vmax = 200 V`, 25%, 50%, 75%, and 100% energy correspond to approximately 100 V, 141.4 V, 173.2 V, and 200 V.

## Development interface

| ESP32-C3 GPIO | Signal |
|---:|---|
| GPIO0 | READ |
| GPIO1 | KICK |
| GPIO2 | FAULT# |
| GPIO3 | DONE# |
| GPIO4 | CHARGE |

This mapping must be checked against the released schematic and board revision.

## Repository map

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
│   └── manufacturing.md
├── firmware/
├── hardware/
│   ├── easyeda/
│   ├── schematic/
│   ├── gerbers/
│   ├── bom/
│   └── pick-and-place/
├── simulations/
├── measurements/
├── media/
└── paper/
```

Binary design files are added only after their exact source and revision are verified. Documentation must never be used as a substitute for a tagged fabrication package.

## Functional validation status

The physical prototype has completed the auxiliary-supply, flyback-enable, capacitor-charge, ready indication, SCR trigger, solenoid actuation, and recharge sequence. Quantitative hardware comparison of voltage, current, efficiency, EMI, and shot repeatability remains pending.

## Paper

Associated manuscript:

**Projeto, Modelagem e Validação Funcional de um Sistema Compacto de Chute Eletromagnético para Robôs da RoboCup Small Size League**

The current consolidated paper explicitly separates simulated quantities, LCR measurements, and functional observations.

## License and citation

See [LICENSE](LICENSE), [CONTRIBUTING.md](CONTRIBUTING.md), and [CITATION.cff](CITATION.cff).

Developed by **TauraBots**, Universidade Federal de Santa Maria (UFSM).
