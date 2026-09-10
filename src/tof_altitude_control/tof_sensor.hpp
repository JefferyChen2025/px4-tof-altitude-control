#pragma once

#include <cstdint>

#include <uORB/Subscription.hpp>
#include <uORB/topics/distance_sensor.h>


// =================================================================
// Common ToF sensor interface
// =================================================================

class ToFSensor
{
public:
    virtual ~ToFSensor() = default;

    virtual bool read(
        float reference_altitude_m,
        float &distance_m
    ) = 0;
};


// =================================================================
// Simulated ToF sensor
// =================================================================

class SimulatedToFSensor : public ToFSensor
{
public:

    SimulatedToFSensor() = default;

    bool read(
        float reference_altitude_m,
        float &distance_m
    ) override;

    void reset();


private:

    float generate_noise();


    int _measurement_counter{0};


    // +/- 3 cm simulated noise
    static constexpr float _noise_amplitude_m{
        0.03f
    };


    // First 20 samples contain no artificial spikes.
    // At 10 Hz this is about 2 seconds.
    static constexpr int _startup_protection_samples{
        20
    };


    // Approximately one spike every 200 samples.
    static constexpr int _spike_probability_denominator{
        200
    };


    static constexpr float _minimum_spike_m{
        0.8f
    };


    static constexpr float _additional_spike_range_m{
        1.0f
    };
};


// =================================================================
// Real PX4 distance_sensor interface
// =================================================================
//
// The PX4 VL53L1X driver performs the actual I2C communication.
//
// This class simply reads the resulting distance_sensor uORB topic.
//

class PX4DistanceToFSensor : public ToFSensor
{
public:

    PX4DistanceToFSensor() = default;


    bool read(
        float reference_altitude_m,
        float &distance_m
    ) override;


    void reset();


private:

    uORB::Subscription _distance_sensor_sub{
        ORB_ID(distance_sensor)
    };


    distance_sensor_s _distance_sensor{};


    bool _measurement_available{
        false
    };


    // Do not use a distance_sensor measurement if it has
    // not been updated for more than 0.5 seconds.
    static constexpr uint64_t _maximum_age_us{
        500000
    };
};
