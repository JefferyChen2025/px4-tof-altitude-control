# PX4 Integration

## Overview

The project is implemented as a custom PX4 module.

Development currently takes place inside:

```text
PX4-Autopilot/src/examples/tof_altitude_control/
```

---

## Module Files

The custom PX4 source directory contains:

```text
tof_altitude_control.cpp
tof_sensor.cpp
tof_sensor.hpp
CMakeLists.txt
Kconfig
module.yaml
```

---

## Build Configuration

The module is enabled for PX4 SITL using:

```text
CONFIG_EXAMPLES_TOF_ALTITUDE_CONTROL=y
```

in the appropriate PX4 board configuration.

PX4's example Kconfig system automatically discovers the module Kconfig file.

---

## CMake Integration

The module is registered with PX4 using `px4_add_module`.

The module contains both the controller implementation and sensor implementation source files.

Example structure:

```cmake
px4_add_module(
    MODULE examples__tof_altitude_control
    MAIN tof_altitude_control

    SRCS
        tof_altitude_control.cpp
        tof_sensor.cpp

    MODULE_CONFIG
        module.yaml
)
```

---

## SITL Build

From the PX4 root directory:

```bash
make px4_sitl
```

To launch Gazebo with the X500 model:

```bash
make px4_sitl gz_x500
```

---

## Module Commands

Start:

```bash
tof_altitude_control start
```

Status:

```bash
tof_altitude_control status
```

Stop:

```bash
tof_altitude_control stop
```

---

## PX4 Parameters

Available controller parameters can be inspected using:

```bash
param show -a TOF_*
```

Parameters include:

```text
TOF_MODE
TOF_ALT
TOF_KP
TOF_KD
TOF_ALPHA
```

---

## uORB Topics

The project interacts with several PX4 uORB topics.

Important topics include:

```text
vehicle_local_position
distance_sensor
offboard_control_mode
trajectory_setpoint
vehicle_command
parameter_update
```

---

## Simulation Sensor Source

Simulation uses:

```text
vehicle_local_position
```

to obtain a reference altitude.

Since PX4 uses NED:

```text
simulated altitude ≈ -z
```

This altitude is passed to the simulated ToF implementation.

---

## Real Sensor Source

Real ToF measurements are expected through:

```text
distance_sensor
```

The physical path is:

```text
VL53L1X
   ↓
Pixhawk I2C
   ↓
PX4 VL53L1X driver
   ↓
distance_sensor uORB
   ↓
custom sensor interface
```

---

## Offboard Control

The custom module publishes PX4 Offboard control messages at approximately:

```text
10 Hz
```

The module establishes a valid Offboard stream before requesting the Offboard mode.

The module controls vertical velocity only.

Conceptually:

```text
vx = 0
vy = 0
vz = custom altitude controller output
```

Unused position and acceleration fields are set to invalid/unused values as required by the implementation.

---

## Hardware Firmware

SITL compilation produces software for the desktop simulator.

Physical deployment requires compiling the same custom module as part of firmware for the specific Pixhawk board.

The workflow is:

```text
Custom source code
      +
PX4 source tree
      ↓
Hardware-specific PX4 build
      ↓
PX4 firmware file
      ↓
Upload to Pixhawk
```

The exact hardware build target will be confirmed against the physical flight controller and PX4 version before firmware flashing.

---

## Verification After Flashing

After hardware firmware installation, the custom module should be verified from the PX4 shell.

For example:

```bash
tof_altitude_control status
```

and:

```bash
param show -a TOF_*
```

Successful recognition of the module and parameters confirms that the custom code has been included in the physical flight-controller firmware.
