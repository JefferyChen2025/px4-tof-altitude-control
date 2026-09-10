# ToF Sensor Processing

## Overview

Real range sensors contain noise, occasional invalid readings, communication delays, and measurement outliers.

The ToF measurement therefore passes through a processing pipeline before it is used by the altitude controller.

```text
Raw Measurement
      ↓
Sensor Read Check
      ↓
Range Validation
      ↓
Jump Validation
      ↓
Low-Pass Filter
      ↓
PD Controller
```

---

## Simulated Sensor

During SITL testing, a simulated ToF sensor is used.

The simulated measurement is based on PX4 local altitude:

```text
altitude ≈ -vehicle_local_position.z
```

Random measurement noise is added.

The current simulation uses approximately:

```text
±0.03 m
```

of measurement variation.

---

## Fault Injection

Artificial measurement spikes are occasionally generated.

A simulated fault may add approximately:

```text
+0.8 m to +1.8 m
```

to a measurement.

Fault injection allows the rejection logic to be tested without requiring physical sensor failures.

Artificial faults are disabled during the first several sensor samples so that an invalid startup value cannot initialize the filter.

---

## Range Validation

Measurements outside the usable ToF range are rejected.

Current approximate limits:

```text
Minimum: 0.05 m
Maximum: 4.00 m
```

The exact usable range may be adjusted after real VL53L1X testing.

---

## Jump Validation

A new reading is compared with the previous valid measurement.

If:

```text
abs(current - previous_valid) > maximum_jump
```

the reading is rejected.

The current jump threshold is approximately:

```text
0.30 m
```

This provides protection against isolated ToF spikes.

---

## Previous-Valid Strategy

An important design decision is that rejected samples do not replace the previous valid measurement.

Example:

```text
Valid:   1.00 m
Fault:   2.30 m  → rejected
Valid:   1.02 m
```

The second valid measurement is compared against:

```text
1.00 m
```

rather than the rejected:

```text
2.30 m
```

This allows immediate recovery following an isolated spike.

---

## Startup Behavior

The filter is initialized only using a valid measurement.

This prevents the following failure mode:

```text
First measurement = artificial spike
        ↓
filter initialized incorrectly
        ↓
normal readings appear to be large jumps
        ↓
all future readings rejected
```

Protecting the initialization stage makes the filtering pipeline much more robust.

---

## Low-Pass Filter

After validation, the measurement is filtered using:

```text
filtered =
    alpha × current_measurement
    +
    (1 - alpha) × previous_filtered
```

The coefficient is configured through:

```text
TOF_ALPHA
```

Default:

```text
0.20
```

---

## Sensor Read Failure vs Invalid Measurement

The implementation distinguishes between two different conditions.

### Sensor Read Failure

Examples:

- no new uORB data
- stale sensor data
- communication failure
- sensor disconnected

The sensor interface returns failure.

### Invalid Measurement

The sensor interface successfully returns a value, but the controller rejects it.

Examples:

- value outside the valid range
- excessive jump from the previous valid measurement
- non-finite numerical value

Separating these cases improves fault diagnosis.

---

## Real Sensor Interface

Real sensor measurements are obtained through PX4:

```text
distance_sensor
```

The custom module does not communicate directly with the VL53L1X hardware.

Instead:

```text
VL53L1X
   ↓
PX4 driver
   ↓
distance_sensor
   ↓
PX4DistanceToFSensor
   ↓
controller
```

This takes advantage of the existing PX4 sensor-driver infrastructure.

---

## Data Freshness

Real ToF messages are checked for age.

A measurement that has not been updated within the allowed time window is treated as stale.

The current design uses a freshness threshold of approximately:

```text
500 ms
```

This prevents old sensor values from being repeatedly interpreted as current measurements.
