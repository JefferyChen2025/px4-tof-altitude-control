# PX4 SITL Simulation

## Purpose

PX4 Software-In-The-Loop simulation is used to develop and validate the ToF altitude controller before deploying it to physical hardware.

The simulation environment currently uses:

```text
PX4 SITL
Gazebo
X500 quadrotor model
```

---

## Starting the Simulation

From the PX4 source directory:

```bash
make px4_sitl gz_x500
```

After PX4 starts, the custom module can be launched from the PX4 shell.

```bash
tof_altitude_control start
```

---

## Simulation Mode

Set:

```bash
param set TOF_MODE 0
```

to use the simulated ToF sensor.

---

## Controller Parameters

Example:

```bash
param set TOF_ALT 1.0
param set TOF_KP 0.8
param set TOF_KD 0.10
param set TOF_ALPHA 0.20
```

---

## Test Sequence

A typical controller test is:

```text
Start PX4 SITL
      ↓
Launch Gazebo
      ↓
Arm aircraft
      ↓
Perform standard takeoff
      ↓
Start ToF controller
      ↓
Observe altitude response
      ↓
Observe artificial sensor faults
      ↓
Verify outlier rejection
      ↓
Stop controller
      ↓
Land aircraft
```

---

## Validated Behaviors

The current simulation has been used to verify:

- simulated ToF measurements
- random measurement noise
- artificial sensor spikes
- spike rejection
- filter recovery
- P control
- PD control
- correct NED velocity sign
- runtime parameter updates
- Offboard vertical control
- short-duration sensor fault handling
- sensor degraded detection
- extended sensor-loss failsafe

---

## Future Simulation Tests

Planned tests include:

```text
Altitude step-response testing
P vs PD comparison
Different Kp values
Different Kd values
Different filter coefficients
Repeated sensor faults
Longer-duration simulations
```

Results will later be compared with physical flight-test data.
