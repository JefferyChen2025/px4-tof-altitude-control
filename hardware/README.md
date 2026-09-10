# Hardware

This directory contains the mechanical and electrical documentation for the physical quadrotor implementation.

The hardware stage of the project includes:

- Pixhawk-based flight controller
- VL53L1X Time-of-Flight sensor
- quadrotor frame
- motors
- ESCs
- GPS
- RC system
- telemetry
- LiPo power system
- custom ToF sensor mount

---

## Planned Directory Contents

```text
hardware/
│
├── README.md
├── wiring_diagram.pdf
└── tof_mount/
    ├── README.md
    ├── tof_mount.step
    ├── tof_mount.stl
    └── dimensions.pdf
```

---

## Wiring Documentation

A wiring diagram will be added after the physical aircraft configuration has been confirmed.

The diagram will document connections between:

```text
Pixhawk
VL53L1X
GPS
RC receiver
telemetry radio
power module
ESCs
motors
battery
```

---

## Development Rule

Hardware wiring will be verified against official component pinouts before power is applied.

Initial system testing will be performed without propellers.
