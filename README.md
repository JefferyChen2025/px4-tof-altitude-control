# PX4 ToF Altitude Control for a Quadrotor

A custom low-altitude control system for a PX4-based quadrotor using a Time-of-Flight (ToF) distance sensor.

The project implements sensor simulation, measurement validation, low-pass filtering, PD altitude control, PX4 Offboard integration, runtime parameter tuning, and sensor-loss failsafe logic in C++.

The current implementation has been validated in PX4 SITL with Gazebo. Hardware integration with a VL53L1X ToF sensor and a Pixhawk-based quadrotor is the next stage.

---

## Project Goal

The goal of this project is to develop a low-altitude control system that uses a downward-facing ToF sensor to regulate quadrotor altitude.

The system is designed so that the same controller can operate using either:

- simulated ToF measurements in PX4 SITL
- real VL53L1X measurements through PX4's `distance_sensor` uORB topic

This allows the control logic to be developed and validated in simulation before being deployed to real hardware.

---

## System Architecture

                     TOF_MODE
                        |
            +-----------+-----------+
            |                       |
            v                       v
   SimulatedToFSensor      PX4DistanceToFSensor
            |                       |
            +-----------+-----------+
                        |
                        v
              Measurement Validation
                        |
                        v
                Low-Pass Filtering
                        |
                        v
                 PD Controller
                        |
                        v
              Vertical Velocity Cmd
                        |
                        v
                PX4 Offboard Mode
                        |
                        v
             PX4 Position Controller
                        |
                        v
              Attitude / Rate Control
                        |
                        v
                 ESCs / Motors
