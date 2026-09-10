# PX4 ToF Altitude Control for a Quadrotor

A custom low-altitude control system for a PX4-based quadrotor using a downward-facing Time-of-Flight (ToF) distance sensor.

This project implements a custom PX4 C++ module for ToF measurement acquisition, signal validation, low-pass filtering, PD altitude control, runtime parameter tuning, PX4 Offboard integration, and sensor-loss failsafe handling.

The software architecture is designed to support both simulated ToF measurements in PX4 SITL and real measurements from a VL53L1X ToF sensor through the PX4 `distance_sensor` uORB topic.

The current software implementation has been validated in PX4 SITL with Gazebo. Physical quadrotor integration and real-sensor testing are the next stages of development.

---

## Project Motivation

Reliable altitude estimation becomes particularly important when a UAV operates close to the ground.

GPS and barometric altitude estimates are useful for general flight but may not provide the most direct measurement of vehicle-to-ground distance during low-altitude operation.

This project investigates a simple alternative: using a downward-facing Time-of-Flight sensor as the primary measurement for an outer-loop low-altitude controller.

The objective is not to replace the complete PX4 flight-control stack.

Instead, the custom controller provides a vertical velocity command while PX4 continues to perform:

- state estimation
- attitude stabilization
- angular-rate control
- control allocation
- motor output generation
- vehicle safety management

This approach allows the project to focus specifically on sensor processing, altitude feedback control, system integration, and fault handling.

---

## System Architecture

```text
                         TOF_MODE
                            |
              +-------------+-------------+
              |                           |
              v                           v
     SimulatedToFSensor          PX4DistanceToFSensor
              |                           |
              +-------------+-------------+
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
               Vertical Velocity Command
                            |
                            v
                   PX4 Offboard Mode
                            |
                            v
              PX4 Position Controller
                            |
                            v
           Attitude / Rate Controllers
                            |
                            v
                 Control Allocation
                            |
                            v
                      ESCs / Motors
```

The same sensor-processing and control pipeline is used regardless of whether the measurement comes from simulation or physical hardware.

---

## Software Architecture

The custom PX4 module is divided into two main parts.

### Controller Module

`tof_altitude_control.cpp`

Responsible for:

- PX4 module lifecycle
- runtime parameters
- sensor selection
- measurement validation
- low-pass filtering
- PD control
- Offboard setpoint publication
- sensor health monitoring
- failsafe handling

### Sensor Abstraction

`tof_sensor.hpp` and `tof_sensor.cpp`

Provide a common ToF sensor interface.

Two implementations are currently supported:

```text
SimulatedToFSensor
PX4DistanceToFSensor
```

`SimulatedToFSensor` generates synthetic measurements for SITL development.

`PX4DistanceToFSensor` receives real range measurements through PX4's `distance_sensor` uORB topic.

---

## Simulated ToF Sensor

The simulated sensor uses PX4 local-position altitude as the reference distance.

Because PX4 uses the NED coordinate convention:

```text
z > 0     downward
z < 0     upward
```

the approximate altitude above the local origin is:

```text
altitude = -local_position.z
```

Synthetic ToF measurements include approximately ±3 cm of random measurement noise.

Artificial range spikes are also occasionally injected to evaluate the robustness of the measurement-validation algorithm.

Startup protection prevents artificial faults from corrupting the initial filter state.

---

## Measurement Validation

Raw ToF readings are validated before entering the controller.

The validation layer checks conditions such as:

- finite numerical value
- minimum sensor range
- maximum sensor range
- maximum change relative to the previous valid reading

The currently designed operating range is approximately:

```text
0.05 m to 4.0 m
```

A measurement with an excessive jump from the previous valid reading is rejected as a potential outlier.

Only valid measurements update:

- the low-pass filter
- the previous-valid-measurement state
- the PD derivative state

This prevents a single faulty measurement from contaminating future controller calculations.

---

## Low-Pass Filtering

Valid ToF measurements are passed through a first-order low-pass filter:

```text
filtered = alpha * measurement
         + (1 - alpha) * previous_filtered
```

where:

```text
alpha = TOF_ALPHA
```

A smaller value of `alpha` produces stronger smoothing but increases measurement delay.

A larger value responds more quickly but allows more sensor noise into the controller.

The default value is:

```text
TOF_ALPHA = 0.20
```

---

## PD Altitude Controller

The desired altitude is defined by:

```text
TOF_ALT
```

Altitude error is calculated as:

```text
error = target_altitude - measured_altitude
```

The controller uses:

```text
u = Kp * error + Kd * d(error)/dt
```

where:

```text
Kp = TOF_KP
Kd = TOF_KD
```

The derivative is calculated using the actual elapsed time between valid measurements rather than assuming a fixed loop period.

This is important because rejected sensor measurements can create irregular intervals between valid controller updates.

The derivative term is disabled for the first valid measurement to avoid an initialization derivative spike.

---

## PX4 Coordinate Convention

The controller internally interprets a positive command as an upward velocity request.

PX4 local velocity uses the NED coordinate system:

```text
negative vz = upward
positive vz = downward
```

Therefore:

```text
PX4 vz setpoint = -upward_velocity_command
```

The vertical velocity command is saturated to prevent excessive control requests.

The current software uses a conservative limit of approximately:

```text
±0.5 m/s
```

---

## PX4 Offboard Integration

The custom module publishes:

- `OffboardControlMode`
- `TrajectorySetpoint`

The module requests velocity-based Offboard control.

Horizontal velocity is held at zero while the custom controller determines the vertical velocity setpoint.

PX4 remains responsible for the lower-level position, attitude, rate, and actuator-control loops.

The custom module continuously publishes the Offboard proof-of-life stream while operating normally.

---

## Runtime Parameters

The project currently defines the following PX4 parameters:

| Parameter | Description | Default |
|---|---|---:|
| `TOF_MODE` | ToF source: 0 = simulation, 1 = real PX4 distance sensor | 0 |
| `TOF_ALT` | Desired altitude | 1.00 m |
| `TOF_KP` | Proportional gain | 0.80 |
| `TOF_KD` | Derivative gain | 0.10 |
| `TOF_ALPHA` | Low-pass filter coefficient | 0.20 |

Example:

```bash
param set TOF_ALT 1.0
param set TOF_KP 0.8
param set TOF_KD 0.10
param set TOF_ALPHA 0.20
```

Parameters can be changed while the module is running.

The controller listens for PX4 parameter updates and refreshes its internal configuration without requiring firmware recompilation.

---

## Sensor Modes

### Simulation Mode

```bash
param set TOF_MODE 0
```

Uses:

```text
SimulatedToFSensor
```

This mode is intended for PX4 SITL development and testing.

---

### Real Sensor Mode

```bash
param set TOF_MODE 1
```

Uses:

```text
PX4DistanceToFSensor
```

The real-sensor interface subscribes to:

```text
distance_sensor
```

and checks the freshness and validity of incoming measurements.

The planned physical sensor is a VL53L1X connected to the Pixhawk through I2C.

---

## Sensor Fault Handling

Sensor reliability is treated separately from measurement filtering.

A sensor read failure may indicate:

- missing data
- stale uORB messages
- communication failure
- disconnected sensor

A successful sensor read can still produce a measurement that fails the controller's range or jump-validation checks.

This distinction allows communication faults and measurement-quality faults to be handled independently.

---

## Temporary Invalid Measurements

A single invalid measurement does not immediately trigger a system shutdown.

During short sensor interruptions:

- the invalid sample is rejected
- filter state is preserved
- derivative state is preserved
- the controller does not reuse a potentially unsafe vertical command
- a zero vertical velocity command is published

This allows the system to recover immediately when valid measurements return.

---

## Sensor Degraded State

If invalid measurements continue for approximately:

```text
0.5 seconds
```

the controller reports a degraded ToF condition.

Example:

```text
ToF DEGRADED
```

This provides an intermediate warning before a complete sensor-loss failsafe occurs.

---

## Sensor-Loss Failsafe

If sensor data remains continuously invalid for approximately:

```text
2 seconds
```

the custom controller declares the ToF sensor lost.

The module exits and stops producing the Offboard proof-of-life stream.

PX4 can then execute its configured Offboard-loss failsafe behavior.

This prevents the custom controller from indefinitely operating without valid altitude feedback.

---

## Building with PX4

The custom module is currently developed inside the PX4 source tree under:

```text
PX4-Autopilot/
└── src/
    └── examples/
        └── tof_altitude_control/
```

The corresponding SITL board configuration enables:

```text
CONFIG_EXAMPLES_TOF_ALTITUDE_CONTROL=y
```

A standard SITL build can be performed with:

```bash
make px4_sitl
```

Gazebo X500 simulation can be started using:

```bash
make px4_sitl gz_x500
```

---

## Running the Module

Start:

```bash
tof_altitude_control start
```

Check status:

```bash
tof_altitude_control status
```

Stop:

```bash
tof_altitude_control stop
```

Display project parameters:

```bash
param show -a TOF_*
```

---

## Development Progress

### Software Completed

- [x] PX4 development environment
- [x] PX4 SITL build
- [x] Gazebo X500 simulation
- [x] custom PX4 C++ module
- [x] simulated ToF sensor
- [x] synthetic measurement noise
- [x] artificial spike injection
- [x] measurement range validation
- [x] measurement jump rejection
- [x] low-pass filtering
- [x] P altitude controller
- [x] PD altitude controller
- [x] PX4 Offboard velocity control
- [x] runtime PX4 parameters
- [x] sensor health monitoring
- [x] degraded sensor detection
- [x] sensor-loss failsafe
- [x] simulated/real sensor abstraction
- [x] PX4 `distance_sensor` interface

### Hardware / Testing Planned

- [ ] assemble physical quadrotor
- [ ] configure Pixhawk
- [ ] integrate VL53L1X through I2C
- [ ] verify `distance_sensor`
- [ ] design ToF CAD mount
- [ ] 3D print sensor mount
- [ ] perform propeller-off integration testing
- [ ] validate RC takeover and failsafes
- [ ] perform standard PX4 hover
- [ ] enable real ToF controller
- [ ] tune PD gains
- [ ] collect flight logs
- [ ] quantify controller performance

---

## Planned Evaluation

Flight-test analysis will evaluate metrics such as:

- altitude tracking error
- steady-state error
- rise time
- overshoot
- settling time
- measurement noise
- response to sensor outliers
- response to temporary measurement loss

A comparison between P-only and PD control is also planned.

---

## Hardware

The physical implementation is designed around a Pixhawk-based PX4 quadrotor.

Major components include:

- PX4-compatible Pixhawk flight controller
- quadrotor frame
- brushless motors and ESCs
- GPS
- RC receiver
- telemetry
- LiPo power system
- VL53L1X ToF distance sensor
- custom 3D-printed downward-facing sensor mount

Exact hardware configuration will be documented after physical assembly and validation.

---

## CAD and Mechanical Integration

A custom ToF sensor mount will be designed for the physical quadrotor.

The mount will prioritize:

- downward sensor alignment
- unobstructed field of view
- mechanical rigidity
- sensor protection
- cable routing
- low mass
- simple installation and removal

CAD and printable files will be stored under:

```text
hardware/tof_mount/
```

---

## Repository Structure

```text
px4-tof-altitude-control/
│
├── README.md
├── src/
│   └── tof_altitude_control/
├── docs/
├── hardware/
├── simulation/
├── experiments/
└── images/
```

---

## Technologies

This project involves:

- C++
- PX4 Autopilot
- uORB
- PX4 parameters
- PX4 Offboard control
- Gazebo
- PX4 SITL
- CMake
- Git
- Linux
- embedded systems
- I2C
- Time-of-Flight sensing
- signal filtering
- feedback control
- PD control
- fault detection
- UAV system integration
- CAD
- 3D printing
- flight testing
- flight-log analysis

---

## Safety

Physical testing will follow a staged validation process.

Initial hardware testing will be performed without propellers.

The following will be verified before flight:

- electrical connections
- Pixhawk configuration
- sensor communication
- RC control
- failsafe behavior
- motor assignment
- motor direction
- custom module operation
- ToF measurement validity

The custom ToF controller will only be tested in flight after normal PX4 flight behavior and manual recovery capability have been verified.

---

## Author

Junfei Chen

Electrical Engineering  
University of Toronto
