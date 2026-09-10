# Hardware Integration Plan

## Objective

The hardware stage will transfer the ToF altitude-control system from PX4 SITL to a physical quadrotor.

The physical vehicle will use a Pixhawk-compatible PX4 flight controller and a downward-facing VL53L1X ToF sensor.

---

## Main Hardware Subsystems

The aircraft consists of several major subsystems:

```text
Battery
   ↓
Power Distribution
   ↓
ESCs
   ↓
Brushless Motors

Battery
   ↓
Power Module
   ↓
Pixhawk

Pixhawk
   ├── GPS
   ├── RC Receiver
   ├── Telemetry
   └── I2C
         ↓
      VL53L1X
```

---

## ToF Sensor

The planned altitude sensor is:

```text
ST VL53L1X Time-of-Flight sensor
```

The sensor will be installed facing vertically downward.

Its purpose is to measure direct vehicle-to-ground distance during low-altitude flight.

---

## Sensor Communication

The VL53L1X communicates using:

```text
I2C
```

Typical I2C signals include:

```text
VCC
GND
SDA
SCL
```

The exact Pixhawk connector pinout will be verified against the flight-controller and sensor documentation before connecting hardware.

No wiring should be performed based only on wire color.

---

## ToF Mount

A custom sensor bracket will be designed and 3D printed.

Design requirements:

```text
Sensor faces vertically downward
Optical window remains unobstructed
Sensor does not move during flight
Cable exits without sharp bending
Mount does not interfere with landing gear
Mount protects the sensor from impact
Mount remains lightweight
```

---

## Initial Hardware Validation

All initial electronics testing will be performed without propellers.

The validation sequence is:

```text
Inspect wiring
      ↓
Check continuity / shorts
      ↓
Power flight controller
      ↓
Verify PX4 boot
      ↓
Verify RC receiver
      ↓
Verify GPS
      ↓
Verify ToF sensor
      ↓
Verify distance_sensor
      ↓
Verify custom module
      ↓
Verify failsafe
      ↓
Motor tests
```

---

## ToF Validation

Before the sensor is used for control, raw measurements will be observed.

Tests should include measurements at known distances such as:

```text
0.25 m
0.50 m
1.00 m
1.50 m
2.00 m
```

Measurements will be checked for:

- offset
- noise
- dropouts
- maximum usable range
- surface-dependent behavior
- update rate
- latency

---

## Custom Module Validation

After `distance_sensor` is confirmed, the module will be switched to:

```bash
param set TOF_MODE 1
```

The following will then be verified without propellers:

```text
real sensor data received
measurement validation works
filter output responds correctly
PD output has correct sign
sensor removal triggers degraded state
extended loss triggers failsafe
```

---

## First Flight Strategy

The first flight will not use the custom ToF controller.

The aircraft will first be tested using standard PX4 flight modes.

The goal is to verify:

- stable takeoff
- stable hover
- correct motor configuration
- acceptable vibration
- RC control
- estimator health
- landing behavior
- failsafe configuration

Only after the aircraft behaves normally will the custom ToF controller be enabled.

---

## Initial ToF Flight Test

The first custom-controller test will use:

```text
low altitude
small vertical commands
conservative gains
manual recovery available
clear test area
```

A target altitude in approximately the:

```text
0.5 m to 1.0 m
```

range is expected to be appropriate for initial testing.

Exact test conditions will be determined after hardware validation.

---

## Flight Data

Flight logs will be recorded for later analysis.

Relevant data will include:

```text
target altitude
ToF altitude
filtered altitude
vertical velocity
controller error
controller output
vehicle position
vehicle velocity
sensor validity
```

The results will be stored under:

```text
experiments/
```
