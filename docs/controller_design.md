# Controller Design

## Objective

The controller attempts to maintain a desired distance between the quadrotor and the ground.

The desired altitude is configured using:

```text
TOF_ALT
```

The measured altitude is obtained from the filtered ToF measurement.

---

## Error Definition

The altitude error is defined as:

```text
error = target_altitude - measured_altitude
```

For example:

```text
Target altitude = 1.00 m
Measured altitude = 0.80 m

Error = +0.20 m
```

A positive error means the vehicle should move upward.

A negative error means the vehicle should move downward.

---

## P Controller

The initial controller used proportional control:

```text
u = Kp × error
```

where:

```text
Kp = TOF_KP
```

The proportional term generates a larger velocity request when the vehicle is farther from the desired altitude.

Advantages:

- simple implementation
- easy tuning
- intuitive behavior

Limitations:

- possible overshoot
- limited damping
- potential steady-state error

---

## PD Controller

The controller was later extended to proportional-derivative control:

```text
u = Kp × error + Kd × error_rate
```

where:

```text
error_rate = Δerror / Δt
```

and:

```text
Kp = TOF_KP
Kd = TOF_KD
```

The derivative term provides damping.

If the vehicle is approaching the target altitude rapidly, the derivative term can oppose the proportional command and reduce overshoot.

---

## Derivative Timing

The derivative term uses the real time interval between valid samples.

Conceptually:

```text
dt = current_valid_timestamp - previous_valid_timestamp
```

This is preferable to assuming a constant loop period because invalid sensor measurements can be rejected.

For example:

```text
Expected loop:
0.10 s
0.10 s
0.10 s

With rejected measurement:
0.10 s
0.20 s
0.10 s
```

Using the actual timestamp prevents incorrect derivative magnitudes after missing measurements.

---

## Derivative Initialization

The derivative term is not calculated on the first valid measurement.

Instead:

```text
D = 0
```

until a previous valid error exists.

This prevents a large artificial derivative spike during controller startup.

---

## Output Saturation

The controller output is limited before being sent to PX4.

Current design:

```text
maximum upward command ≈ +0.5 m/s
maximum downward command ≈ -0.5 m/s
```

Saturation prevents an unusually large altitude error or sensor transient from requesting an excessive vertical velocity.

---

## PX4 Velocity Conversion

The internal controller convention is:

```text
positive = upward
negative = downward
```

PX4 uses NED:

```text
negative vz = upward
positive vz = downward
```

Therefore:

```text
vz_px4 = -u
```

---

## Current Default Gains

The current simulation parameters are:

```text
TOF_KP = 0.80
TOF_KD = 0.10
```

These values are intended as initial values only.

Real-flight controller tuning will be performed after the hardware system has been validated.

---

## Future Controller Evaluation

The controller will be evaluated using metrics such as:

```text
Rise time
Overshoot
Settling time
Steady-state error
Mean altitude error
Maximum altitude error
```

P-only and PD control may also be compared experimentally.

---

## Possible Future Improvements

Potential extensions include:

- integral control
- derivative filtering
- gain scheduling
- ground-effect compensation
- adaptive filtering
- sensor fusion
- acceleration-limited setpoints
- state-machine-based landing behavior

These features are outside the current project scope but provide directions for future work.
