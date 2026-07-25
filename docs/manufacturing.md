# Manufacturing release checklist

A tagged fabrication release is considered complete only when all applicable items below are included and mutually consistent.

## Required files

- EasyEDA source project or exported source archive;
- schematic PDF;
- PCB source and board outline;
- Gerber files;
- plated and non-plated drill files;
- BOM with manufacturer part numbers;
- pick-and-place/CPL file;
- top and bottom assembly drawings;
- layer stack and copper weight;
- board thickness and finish;
- controlled-impedance notes, if applicable;
- fabrication notes and panelization constraints;
- board revision identifier.

## Review points

- high-voltage creepage and clearance;
- connector pinout and polarity;
- transformer orientation and winding identification;
- diode, capacitor, and semiconductor polarity;
- pulse-current paths and copper width;
- thermal relief and heat spreading;
- test points for input, output, feedback, DONE#, FAULT#, CHARGE, and KICK;
- discharge path and safe-state behavior;
- component voltage, current, pulse-energy, and temperature ratings;
- silkscreen warnings and revision marking.

## Release naming

Recommended format:

```text
KickerBoard_revX.Y_fabrication_YYYY-MM-DD.zip
```

The archive should include a manifest with hashes. Do not fabricate from an untagged development directory or from screenshots.
