# VL53L1X ToF Sensor Mount

## Objective

This directory contains the custom CAD design for mounting the VL53L1X Time-of-Flight sensor on the quadrotor.

The sensor will face vertically downward and provide vehicle-to-ground distance measurements for the custom altitude controller.

---

## Design Requirements

The mount should provide:

- vertical downward alignment
- unobstructed optical field of view
- rigid sensor retention
- low weight
- mechanical protection
- easy installation
- easy sensor removal
- suitable cable routing
- compatibility with the quadrotor frame

---

## Design Process

The planned workflow is:

```text
Measure sensor dimensions
        ↓
Measure mounting area
        ↓
Create CAD model
        ↓
Check sensor field of view
        ↓
3D print prototype
        ↓
Install on aircraft
        ↓
Verify alignment
        ↓
Revise design if necessary
```

---

## Planned Files

```text
tof_mount.step
tof_mount.stl
dimensions.pdf
```

The STEP file will provide an editable CAD model.

The STL file will provide a directly printable version.

---

## Printing

The first prototype may be produced using PLA+ or PETG.

The final material and print settings will be selected after evaluating:

- rigidity
- vibration
- weight
- durability
- ease of printing

---

## Sensor Alignment

Incorrect sensor alignment can introduce systematic distance error.

The optical axis should therefore be approximately perpendicular to the ground when the vehicle is level.

The final design will also ensure that the sensor field of view is not blocked by:

- frame components
- landing gear
- wires
- battery
- mounting hardware
