#include "tof_sensor.hpp"

#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>

#include <cstdlib>
#include <cmath>


// =================================================================
// SIMULATED SENSOR
// =================================================================

float SimulatedToFSensor::generate_noise()
{
    const float normalized =
        2.0f
        * static_cast<float>(std::rand())
        / static_cast<float>(RAND_MAX)
        - 1.0f;


    return normalized
           * _noise_amplitude_m;
}


void SimulatedToFSensor::reset()
{
    _measurement_counter = 0;
}


bool SimulatedToFSensor::read(
    float reference_altitude_m,
    float &distance_m)
{
    // Normal measurement + noise
    distance_m =
        reference_altitude_m
        + generate_noise();


    _measurement_counter++;


    // =============================================================
    // Startup protection
    // =============================================================

    if (
        _measurement_counter
        <= _startup_protection_samples
    ) {

        return true;
    }


    // =============================================================
    // Artificial single-sample spike
    // =============================================================

    if (
        (
            std::rand()
            % _spike_probability_denominator
        )
        == 0
    ) {

        const float random_fraction =
            static_cast<float>(
                std::rand()
            )
            / static_cast<float>(
                RAND_MAX
            );


        const float spike =
            _minimum_spike_m
            + random_fraction
            * _additional_spike_range_m;


        distance_m += spike;


        PX4_WARN(
            "Injected ToF spike: +%.2f m",
            (double)spike
        );
    }


    return true;
}


// =================================================================
// REAL PX4 DISTANCE SENSOR
// =================================================================

void PX4DistanceToFSensor::reset()
{
    _distance_sensor = {};
    _measurement_available = false;
}


bool PX4DistanceToFSensor::read(
    float reference_altitude_m,
    float &distance_m)
{
    // Real sensor does not need PX4 ground-truth altitude.
    (void)reference_altitude_m;


    // =============================================================
    // Check for a new distance_sensor message
    // =============================================================

    distance_sensor_s new_measurement{};


    if (
        _distance_sensor_sub.update(
            &new_measurement
        )
    ) {

        _distance_sensor =
            new_measurement;


        _measurement_available =
            true;
    }


    // No distance_sensor message has ever been received.
    if (!_measurement_available) {

        return false;
    }


    // =============================================================
    // Reject stale sensor data
    // =============================================================

    const uint64_t now =
        hrt_absolute_time();


    if (
        now > _distance_sensor.timestamp
        && (
            now
            - _distance_sensor.timestamp
        ) > _maximum_age_us
    ) {

        return false;
    }


    // =============================================================
    // Reject non-finite measurements
    // =============================================================

    if (
        !std::isfinite(
            _distance_sensor.current_distance
        )
    ) {

        return false;
    }


    distance_m =
        _distance_sensor.current_distance;


    return true;
}
