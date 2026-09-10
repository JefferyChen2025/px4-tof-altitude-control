# Failsafe Design

## Purpose

An altitude controller that depends on a range sensor should not continue issuing normal control commands indefinitely when the sensor becomes unavailable.

This project therefore includes a dedicated ToF sensor-health system.

---

## Fault Categories

Sensor faults can include:

- isolated measurement outliers
- temporary communication interruption
- invalid distance values
- stale sensor messages
- disconnected sensor
- failed PX4 sensor driver

Different fault durations are handled differently.

---

## Normal State

When valid ToF measurements are available:

```text
Sensor
  ↓
Validation
  ↓
Filter
  ↓
PD Controller
  ↓
PX4 Offboard Setpoint
```

Normal altitude control continues.

---

## Single Invalid Measurement

A single invalid measurement is rejected.

The controller does not:

- update the filter
- update the previous-valid measurement
- update derivative history

This prevents one bad measurement from corrupting future control calculations.

---

## Temporary Sensor Interruption

During a short period without valid sensor data, the controller does not continue using the previous vertical velocity command.

Instead:

```text
vertical velocity setpoint = 0
```

The Offboard heartbeat continues.

The objective is to avoid blindly continuing a potentially significant climb or descent command while temporarily lacking altitude feedback.

---

## Degraded State

If invalid sensor data continues for approximately:

```text
0.5 seconds
```

the controller enters a degraded sensor condition.

A warning is generated:

```text
ToF DEGRADED
```

At this stage the controller still allows recovery if valid measurements return.

---

## Sensor-Lost State

If continuous sensor failure reaches approximately:

```text
2 seconds
```

the controller declares:

```text
ToF SENSOR LOST
```

The custom module exits.

---

## Interaction with PX4 Failsafe

When the custom module exits, it no longer publishes the Offboard proof-of-life signal.

PX4 can detect the loss of the Offboard control source and perform its configured Offboard-loss behavior.

Conceptually:

```text
ToF lost
   ↓
Custom controller exits
   ↓
Offboard heartbeat stops
   ↓
PX4 detects Offboard loss
   ↓
PX4 configured failsafe action
```

The final physical-aircraft failsafe behavior will be configured and verified before custom-controller flight testing.

---

## Local Position Failure

The controller also depends on valid PX4 vehicle state information.

If required PX4 local-position data becomes invalid, the custom controller should not continue normal operation.

This condition therefore also results in controller shutdown.

---

## Design Principle

The custom module does not attempt to replace PX4's full vehicle safety system.

Instead, it detects failures specific to the custom ToF controller and then returns authority to PX4's existing safety infrastructure.

This maintains a clear boundary between:

```text
Custom project-specific safety logic
```

and:

```text
PX4 vehicle-level failsafe logic
```

---

## Planned Hardware Tests

Failsafe behavior will be validated with the propellers removed.

Tests will include:

```text
Covering the ToF sensor
Disconnecting the ToF sensor
Out-of-range measurements
Stale sensor data
Temporary measurement interruption
Extended sensor interruption
```

Flight testing will begin only after these behaviors have been verified on the ground.
