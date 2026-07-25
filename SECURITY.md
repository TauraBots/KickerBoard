# Security and safety reporting

KickerBoard is a high-voltage hardware project. Reports may concern software security, unsafe state transitions, electrical hazards, manufacturing defects, or documentation that could cause misuse.

## Report privately when possible

Do not publish an exploitable or immediately hazardous issue before maintainers have had an opportunity to assess it. Include:

- affected board, firmware, or document revision;
- reproduction conditions;
- expected and observed behavior;
- photographs, logs, or waveforms where safe to obtain;
- potential consequence;
- suggested mitigation, if known.

## Examples

- charging remains active after FAULT#;
- KICK becomes active during boot or reset;
- automatic firing can occur without explicit arming;
- voltage feedback can be interpreted outside a safe range;
- fabrication data has a polarity, clearance, or connector error;
- documentation incorrectly labels simulated data as measured.

## Immediate response

Stop using affected hardware, isolate the input supply, and follow the documented discharge-verification procedure. Do not continue testing a suspected unsafe revision merely to collect additional evidence.

## Supported versions

Only tagged releases explicitly marked verified are supported. Development branches and untagged manufacturing packages must be treated as experimental.
