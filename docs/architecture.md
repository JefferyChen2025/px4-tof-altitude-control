# System Architecture

## Overview

This project implements a custom outer-loop altitude controller for a PX4-based quadrotor.

A downward-facing Time-of-Flight sensor measures vehicle-to-ground distance.

The measurement is validated, filtered, and processed by a PD controller.

The resulting control output is converted into a PX4 vertical velocity setpoint.

PX4 remains responsible for low-level flight stabilization.

---

## High-Level Architecture

```text
                            PX4
                             |
                      Local Position
                             |
                             v
                  SimulatedToFSensor
                         (SITL)
                             |
                             |
             TOF_MODE -------+
                             |
                             v
                       ToFSensor
                             ^
                             |
                  PX4DistanceToFSensor
                             ^
                             |
                    distance_sensor
                             ^
                             |
                       VL53L1X
                      Real Hardware

                             |
                             v
                  Measurement Validation
                             |
                             v
                      Low-Pass Filter
                             |
                             v
                       PD Controller
                             |
                             v
                Vertical Velocity Setpoint
                             |
                             v
                    PX4 Offboard API
                             |
                             v
                  PX4 Position Control
                             |
                             v
                 Attitude / Rate Control
                             |
                             v
                     ESCs / Motors
```

---

## Design Philosophy

The project deliberately does not replace PX4's complete flight-control system.

The custom module operates at a higher control level.

Its responsibility is to determine an appropriate vertical velocity command based on the measured ground distance.

PX4 continues to provide:

- vehicle state estimation
- attitude stabilization
- angular-rate stabilization
- control allocation
- actuator outputs
- flight modes
- arming logic
- system failsafes

This separation reduces development risk and allows the project to focus on ToF sensing and low-altitude control.

---

## Sensor Abstraction

A common sensor interface allows the controller to operate with multiple measurement sources.

Conceptually:

```cpp
class ToFSensor
{
public:
    virtual bool read(
        float reference_altitude_m,
        float &distance_m
    ) = 0;
};
```

Two implementations are currently used:

```text
SimulatedToFSensor
PX4DistanceToFSensor
```

The controller therefore does not need separate control algorithms for simulation and real hardware.

---

## Simulation Path

In SITL:

```text
PX4 vehicle_local_position
        ↓
altitude = -z
        ↓
SimulatedToFSensor
        ↓
noise + artificial faults
        ↓
common processing pipeline
```

This allows sensor-processing and control logic to be tested before physical hardware is available.

---

## Real Hardware Path

On the physical vehicle:

```text
VL53L1X
   ↓
I2C
   ↓
PX4 sensor driver
   ↓
distance_sensor uORB topic
   ↓
PX4DistanceToFSensor
   ↓
common processing pipeline
```

This architecture minimizes changes between simulation and hardware deployment.

---

## Controller Path

The controller pipeline is:

```text
sensor read
    ↓
freshness / communication check
    ↓
measurement validity check
    ↓
low-pass filtering
    ↓
altitude error
    ↓
PD controller
    ↓
velocity saturation
    ↓
NED coordinate conversion
    ↓
PX4 TrajectorySetpoint
```

---

## Coordinate System

PX4 uses the North-East-Down coordinate convention.

For the vertical axis:

```text
+z = downward
-z = upward
```

Similarly:

```text
+vz = downward velocity
-vz = upward velocity
```

The custom controller calculates a positive command as upward motion and therefore publishes:

```text
vz_px4 = -upward_velocity_command
```

---

## Safety Architecture

Sensor faults are handled at multiple levels.

```text
Valid measurement
      ↓
normal control

Single invalid measurement
      ↓
reject sample
      ↓
zero vertical velocity

Continuous invalid data
      ↓
degraded state

Extended sensor loss
      ↓
custom module exits
      ↓
Offboard heartbeat stops
      ↓
PX4 Offboard-loss failsafe
```

This ensures the custom controller does not continue operating indefinitely without valid altitude feedback.
