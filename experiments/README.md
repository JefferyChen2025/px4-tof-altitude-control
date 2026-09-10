# Experiments and Flight-Test Analysis

## Purpose

This directory stores data and analysis produced during simulation and physical flight testing.

The objective is to quantitatively evaluate the performance of the ToF altitude controller.

---

## Directory Structure

```text
experiments/
│
├── README.md
├── raw_data/
├── processed_data/
└── plots/
```

---

## Raw Data

`raw_data/` is intended for unprocessed experimental measurements.

Possible data sources include:

```text
PX4 ULog files
ToF measurements
controller debug logs
simulation logs
CSV exports
```

Large raw files may not be stored directly in the repository.

---

## Processed Data

`processed_data/` will contain cleaned or converted datasets used for analysis.

Examples:

```text
altitude_response.csv
sensor_noise.csv
step_response.csv
p_vs_pd.csv
```

---

## Plots

`plots/` will contain final figures used in project documentation.

Planned plots include:

```text
Target altitude vs measured altitude
Raw ToF vs filtered ToF
Altitude error vs time
Vertical velocity command vs time
P vs PD response comparison
Sensor-noise distribution
Outlier rejection example
```

---

## Performance Metrics

Controller performance will be evaluated using metrics such as:

### Rise Time

Time required for the vehicle to approach the target altitude after a command change.

### Overshoot

Maximum amount by which altitude exceeds the desired value.

### Settling Time

Time required for altitude to remain within a defined tolerance around the target.

### Steady-State Error

Difference between target altitude and measured altitude after the transient response has ended.

### Mean Absolute Error

```text
MAE = mean(|target - measured|)
```

This provides an overall measure of altitude tracking accuracy.

---

## Planned Experiments

### Experiment 1 — Stationary ToF Noise

Measure the sensor while the vehicle remains stationary.

Objective:

```text
characterize raw sensor noise
```

---

### Experiment 2 — Filter Comparison

Compare:

```text
Raw ToF
vs
Low-pass filtered ToF
```

Objective:

```text
evaluate noise reduction and filtering delay
```

---

### Experiment 3 — P Controller

Run altitude tracking using proportional control.

Measure:

```text
rise time
overshoot
settling time
steady-state error
```

---

### Experiment 4 — PD Controller

Repeat the test using derivative damping.

Compare the result with P-only control.

---

### Experiment 5 — Sensor Fault

Introduce temporary ToF measurement interruption.

Verify:

```text
invalid measurement rejection
degraded state
sensor-loss failsafe
recovery behavior
```

---

## Experimental Safety

Physical flight experiments will only begin after:

```text
standard PX4 hover is verified
RC takeover is verified
sensor readings are validated
controller sign is validated
failsafe behavior is validated
```
```

Results will later be compared with physical flight-test data.
