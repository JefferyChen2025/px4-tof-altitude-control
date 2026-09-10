#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/parameter_update.h>

#include "tof_sensor.hpp"

#include <atomic>
#include <cstring>
#include <cmath>
#include <cstdint>


class ToFAltitudeControl : public ModuleParams
{
public:

    ToFAltitudeControl() :
        ModuleParams(nullptr)
    {
    }


    int run()
    {
        PX4_INFO(
            "ToF PD altitude controller started"
        );


        // Load PX4 parameters.
        updateParams();


        _current_sensor_mode =
            _param_tof_mode.get();


        reset_sensor_processing_state();


        if (_current_sensor_mode == 0) {

            PX4_INFO(
                "Sensor mode: SIMULATION"
            );

        } else {

            PX4_INFO(
                "Sensor mode: REAL distance_sensor"
            );
        }


        // =========================================================
        // PX4 local position subscription
        // =========================================================

        uORB::Subscription local_position_sub{
            ORB_ID(vehicle_local_position)
        };


        vehicle_local_position_s local_position{};


        int loop_counter = 0;


        // =========================================================
        // Main loop
        // =========================================================

        while (!_should_exit.load()) {

            // =====================================================
            // 1. Runtime PX4 parameter updates
            // =====================================================

            update_parameters();


            // =====================================================
            // 2. Detect sensor-mode change
            // =====================================================

            const int requested_sensor_mode =
                _param_tof_mode.get();


            if (
                requested_sensor_mode
                != _current_sensor_mode
            ) {

                handle_sensor_mode_change(
                    requested_sensor_mode
                );
            }


            // =====================================================
            // 3. Read PX4 local position
            // =====================================================

            if (
                local_position_sub.update(
                    &local_position
                )
            ) {

                if (!local_position.z_valid) {

                    PX4_ERR(
                        "PX4 local altitude invalid"
                    );


                    _failsafe_triggered =
                        true;


                    break;
                }


                // PX4 NED:
                // positive z = downward
                //
                // Positive-up altitude = -z

                const float true_altitude =
                    -local_position.z;


                // =================================================
                // 4. Read selected ToF source
                // =================================================

                float raw_tof =
                    0.0f;


                const bool sensor_read_success =
                    read_selected_sensor(
                        true_altitude,
                        raw_tof
                    );


                // =================================================
                // 5. Sensor read failure
                // =================================================

                if (!sensor_read_success) {

                    handle_invalid_measurement(
                        true_altitude,
                        raw_tof,
                        loop_counter,
                        "READ FAIL"
                    );


                    if (_failsafe_triggered) {
                        break;
                    }


                    loop_counter++;


                    px4_usleep(
                        100000
                    );


                    continue;
                }


                // =================================================
                // 6. Measurement validity
                // =================================================

                const bool measurement_valid =
                    is_measurement_valid(
                        raw_tof
                    );


                if (measurement_valid) {

                    // =============================================
                    // Sensor recovery
                    // =============================================

                    if (
                        _consecutive_invalid_count
                        > 0
                    ) {

                        PX4_INFO(
                            "ToF recovered after %d invalid samples",
                            _consecutive_invalid_count
                        );
                    }


                    _consecutive_invalid_count =
                        0;


                    _invalid_start_timestamp =
                        0;


                    _degraded_warning_sent =
                        false;


                    // =============================================
                    // 7. Low-pass filter
                    // =============================================

                    if (!_filter_initialized) {

                        _filtered_tof =
                            raw_tof;


                        _filter_initialized =
                            true;

                    } else {

                        const float alpha =
                            _param_tof_alpha.get();


                        _filtered_tof =
                            alpha
                            * raw_tof
                            + (1.0f - alpha)
                            * _filtered_tof;
                    }


                    // =============================================
                    // 8. Target altitude
                    // =============================================

                    const float target_altitude =
                        _param_tof_alt.get();


                    // =============================================
                    // 9. Altitude error
                    // =============================================

                    const float altitude_error =
                        target_altitude
                        - _filtered_tof;


                    // =============================================
                    // 10. P term
                    // =============================================

                    const float p_term =
                        _param_tof_kp.get()
                        * altitude_error;


                    // =============================================
                    // 11. D term
                    // =============================================

                    float error_rate =
                        0.0f;


                    float d_term =
                        0.0f;


                    const uint64_t now =
                        hrt_absolute_time();


                    if (
                        _previous_error_available
                    ) {

                        const float dt =
                            static_cast<float>(
                                now
                                - _previous_error_timestamp
                            )
                            / 1000000.0f;


                        if (dt > 0.001f) {

                            error_rate =
                                (
                                    altitude_error
                                    - _previous_error
                                )
                                / dt;


                            d_term =
                                _param_tof_kd.get()
                                * error_rate;
                        }
                    }


                    // =============================================
                    // 12. PD output
                    // =============================================

                    float upward_velocity_command =
                        p_term
                        + d_term;


                    // =============================================
                    // 13. Saturation
                    // =============================================

                    if (
                        upward_velocity_command
                        > _max_vertical_speed
                    ) {

                        upward_velocity_command =
                            _max_vertical_speed;
                    }


                    if (
                        upward_velocity_command
                        < -_max_vertical_speed
                    ) {

                        upward_velocity_command =
                            -_max_vertical_speed;
                    }


                    // =============================================
                    // 14. Convert to PX4 NED
                    // =============================================
                    //
                    // Controller:
                    // + = upward
                    //
                    // PX4 NED:
                    // -vz = upward

                    const float vz_ned =
                        -upward_velocity_command;


                    // =============================================
                    // 15. Publish Offboard
                    // =============================================

                    publish_offboard_control_mode();


                    publish_velocity_setpoint(
                        vz_ned
                    );


                    // =============================================
                    // 16. Request OFFBOARD
                    // =============================================

                    if (!_offboard_requested) {

                        _offboard_counter++;


                        // 15 cycles at 10 Hz ≈ 1.5 seconds.
                        if (
                            _offboard_counter
                            >= 15
                        ) {

                            request_offboard_mode();


                            _offboard_requested =
                                true;


                            PX4_INFO(
                                "Requested OFFBOARD mode"
                            );
                        }
                    }


                    // =============================================
                    // 17. Save previous VALID measurement
                    // =============================================

                    _previous_valid_tof =
                        raw_tof;


                    _previous_valid_measurement_available =
                        true;


                    // =============================================
                    // 18. Save derivative state
                    // =============================================

                    _previous_error =
                        altitude_error;


                    _previous_error_timestamp =
                        now;


                    _previous_error_available =
                        true;


                    // =============================================
                    // 19. Logging
                    // =============================================

                    if (
                        loop_counter % 10
                        == 0
                    ) {

                        PX4_INFO(
                            "Mode %d | Target %.2f | Alt %.2f",
                            _current_sensor_mode,
                            (double)target_altitude,
                            (double)true_altitude
                        );


                        PX4_INFO(
                            "ToF %.3f | Filt %.3f | Err %.3f",
                            (double)raw_tof,
                            (double)_filtered_tof,
                            (double)altitude_error
                        );


                        PX4_INFO(
                            "P %.3f | D %.3f | dErr %.3f",
                            (double)p_term,
                            (double)d_term,
                            (double)error_rate
                        );


                        PX4_INFO(
                            "UpCmd %.3f | PX4 vz %.3f | VALID",
                            (double)upward_velocity_command,
                            (double)vz_ned
                        );
                    }


                } else {

                    // =============================================
                    // Measurement invalid
                    // =============================================

                    handle_invalid_measurement(
                        true_altitude,
                        raw_tof,
                        loop_counter,
                        "INVALID"
                    );


                    if (_failsafe_triggered) {
                        break;
                    }
                }
            }


            loop_counter++;


            // 10 Hz
            px4_usleep(
                100000
            );
        }


        // =========================================================
        // Exit
        // =========================================================
        //
        // Do not continue sending Offboard heartbeat after exit.

        if (_failsafe_triggered) {

            PX4_ERR(
                "ToF controller exited due to FAILSAFE"
            );

        } else {

            PX4_INFO(
                "ToF PD altitude controller stopped"
            );
        }


        return 0;
    }


    void request_stop()
    {
        _should_exit.store(
            true
        );
    }


private:

    // =============================================================
    // Read currently selected sensor
    // =============================================================

    bool read_selected_sensor(
        float true_altitude,
        float &distance_m)
    {
        if (_current_sensor_mode == 0) {

            return _simulated_sensor.read(
                true_altitude,
                distance_m
            );
        }


        if (_current_sensor_mode == 1) {

            return _real_sensor.read(
                true_altitude,
                distance_m
            );
        }


        return false;
    }


    // =============================================================
    // Handle sensor-mode change
    // =============================================================

    void handle_sensor_mode_change(
        int new_mode)
    {
        if (
            new_mode < 0
            || new_mode > 1
        ) {

            PX4_ERR(
                "Invalid TOF_MODE: %d",
                new_mode
            );


            return;
        }


        PX4_INFO(
            "Changing sensor mode %d -> %d",
            _current_sensor_mode,
            new_mode
        );


        _current_sensor_mode =
            new_mode;


        // VERY IMPORTANT:
        //
        // The new sensor must not inherit filtering,
        // jump-detection or derivative history from the
        // previous sensor.

        reset_sensor_processing_state();


        if (_current_sensor_mode == 0) {

            PX4_INFO(
                "Sensor mode: SIMULATION"
            );

        } else {

            PX4_INFO(
                "Sensor mode: REAL distance_sensor"
            );
        }
    }


    // =============================================================
    // Reset sensor-dependent processing state
    // =============================================================

    void reset_sensor_processing_state()
    {
        // Reset simulated sensor startup/fault state.
        _simulated_sensor.reset();


        // Reset real sensor cached message.
        _real_sensor.reset();


        // Filter state.
        _filter_initialized =
            false;


        _filtered_tof =
            0.0f;


        // Previous valid reading.
        _previous_valid_measurement_available =
            false;


        _previous_valid_tof =
            0.0f;


        // Derivative state.
        _previous_error_available =
            false;


        _previous_error =
            0.0f;


        _previous_error_timestamp =
            0;


        // Sensor-health state.
        _consecutive_invalid_count =
            0;


        _invalid_start_timestamp =
            0;


        _degraded_warning_sent =
            false;
    }


    // =============================================================
    // PX4 parameter updates
    // =============================================================

    void update_parameters()
    {
        if (
            _parameter_update_sub.updated()
        ) {

            parameter_update_s parameter_update{};


            _parameter_update_sub.copy(
                &parameter_update
            );


            updateParams();


            PX4_INFO(
                "Params MODE %d ALT %.2f KP %.2f KD %.2f",
                (int)_param_tof_mode.get(),
                (double)_param_tof_alt.get(),
                (double)_param_tof_kp.get(),
                (double)_param_tof_kd.get()
            );


            PX4_INFO(
                "ALPHA %.2f",
                (double)_param_tof_alpha.get()
            );
        }
    }


    // =============================================================
    // Measurement validity
    // =============================================================

    bool is_measurement_valid(
        float measurement)
    {
        if (
            !std::isfinite(
                measurement
            )
        ) {

            return false;
        }


        if (
            measurement
            < _min_range
        ) {

            return false;
        }


        if (
            measurement
            > _max_range
        ) {

            return false;
        }


        // Compare only against the previous VALID measurement.
        if (
            _previous_valid_measurement_available
        ) {

            const float jump =
                std::fabs(
                    measurement
                    - _previous_valid_tof
                );


            if (
                jump
                > _max_jump
            ) {

                return false;
            }
        }


        return true;
    }


    // =============================================================
    // Invalid measurement / sensor loss
    // =============================================================

    void handle_invalid_measurement(
        float true_altitude,
        float raw_tof,
        int loop_counter,
        const char *reason)
    {
        const uint64_t now =
            hrt_absolute_time();


        _consecutive_invalid_count++;


        if (
            _invalid_start_timestamp
            == 0
        ) {

            _invalid_start_timestamp =
                now;
        }


        const float invalid_duration =
            static_cast<float>(
                now
                - _invalid_start_timestamp
            )
            / 1000000.0f;


        // Keep Offboard alive temporarily,
        // but command zero vertical velocity.
        publish_offboard_control_mode();


        publish_velocity_setpoint(
            0.0f
        );


        if (
            _consecutive_invalid_count
            == 1
        ) {

            PX4_WARN(
                "ToF %s | Mode %d | True %.3f Raw %.3f",
                reason,
                _current_sensor_mode,
                (double)true_altitude,
                (double)raw_tof
            );
        }


        if (
            invalid_duration
            >= _degraded_timeout_s
            && !_degraded_warning_sent
        ) {

            PX4_WARN(
                "ToF DEGRADED: invalid for %.2f s",
                (double)invalid_duration
            );


            _degraded_warning_sent =
                true;
        }


        if (
            _consecutive_invalid_count
            > 1
            && loop_counter % 10
            == 0
        ) {

            PX4_WARN(
                "ToF still invalid: %.2f s",
                (double)invalid_duration
            );
        }


        if (
            invalid_duration
            >= _sensor_loss_timeout_s
        ) {

            PX4_ERR(
                "ToF SENSOR LOST: %.2f s",
                (double)invalid_duration
            );


            PX4_ERR(
                "Triggering controller failsafe"
            );


            _failsafe_triggered =
                true;


            _should_exit.store(
                true
            );
        }
    }


    // =============================================================
    // Offboard control mode
    // =============================================================

    void publish_offboard_control_mode()
    {
        offboard_control_mode_s msg{};


        msg.timestamp =
            hrt_absolute_time();


        msg.position =
            false;


        msg.velocity =
            true;


        msg.acceleration =
            false;


        msg.attitude =
            false;


        msg.body_rate =
            false;


        msg.thrust_and_torque =
            false;


        msg.direct_actuator =
            false;


        _offboard_control_mode_pub.publish(
            msg
        );
    }


    // =============================================================
    // Velocity setpoint
    // =============================================================

    void publish_velocity_setpoint(
        float vz_ned)
    {
        trajectory_setpoint_s msg{};


        msg.timestamp =
            hrt_absolute_time();


        msg.position[0] =
            NAN;

        msg.position[1] =
            NAN;

        msg.position[2] =
            NAN;


        msg.velocity[0] =
            0.0f;

        msg.velocity[1] =
            0.0f;

        msg.velocity[2] =
            vz_ned;


        msg.acceleration[0] =
            NAN;

        msg.acceleration[1] =
            NAN;

        msg.acceleration[2] =
            NAN;


        msg.jerk[0] =
            NAN;

        msg.jerk[1] =
            NAN;

        msg.jerk[2] =
            NAN;


        msg.yaw =
            NAN;


        msg.yawspeed =
            NAN;


        _trajectory_setpoint_pub.publish(
            msg
        );
    }


    // =============================================================
    // Request OFFBOARD
    // =============================================================

    void request_offboard_mode()
    {
        vehicle_command_s msg{};


        msg.timestamp =
            hrt_absolute_time();


        msg.command =
            vehicle_command_s::
            VEHICLE_CMD_DO_SET_MODE;


        msg.param1 =
            1.0f;


        // Main mode 6 = OFFBOARD
        msg.param2 =
            6.0f;


        msg.target_system =
            1;


        msg.target_component =
            1;


        msg.source_system =
            1;


        msg.source_component =
            1;


        msg.from_external =
            true;


        _vehicle_command_pub.publish(
            msg
        );
    }


private:

    // =============================================================
    // Sensors
    // =============================================================

    SimulatedToFSensor
        _simulated_sensor{};


    PX4DistanceToFSensor
        _real_sensor{};


    int _current_sensor_mode{
        0
    };


    // =============================================================
    // Module state
    // =============================================================

    std::atomic_bool _should_exit{
        false
    };


    bool _failsafe_triggered{
        false
    };


    // =============================================================
    // Parameter subscription
    // =============================================================

    uORB::Subscription _parameter_update_sub{
        ORB_ID(parameter_update)
    };


    // =============================================================
    // uORB publications
    // =============================================================

    uORB::Publication<offboard_control_mode_s>
    _offboard_control_mode_pub{
        ORB_ID(offboard_control_mode)
    };


    uORB::Publication<trajectory_setpoint_s>
    _trajectory_setpoint_pub{
        ORB_ID(trajectory_setpoint)
    };


    uORB::Publication<vehicle_command_s>
    _vehicle_command_pub{
        ORB_ID(vehicle_command)
    };


    // =============================================================
    // Filter state
    // =============================================================

    bool _filter_initialized{
        false
    };


    float _filtered_tof{
        0.0f
    };


    // =============================================================
    // Previous VALID ToF
    // =============================================================

    bool _previous_valid_measurement_available{
        false
    };


    float _previous_valid_tof{
        0.0f
    };


    // =============================================================
    // PD derivative state
    // =============================================================

    bool _previous_error_available{
        false
    };


    float _previous_error{
        0.0f
    };


    uint64_t _previous_error_timestamp{
        0
    };


    // =============================================================
    // Offboard state
    // =============================================================

    int _offboard_counter{
        0
    };


    bool _offboard_requested{
        false
    };


    // =============================================================
    // Sensor-health state
    // =============================================================

    int _consecutive_invalid_count{
        0
    };


    uint64_t _invalid_start_timestamp{
        0
    };


    bool _degraded_warning_sent{
        false
    };


    // =============================================================
    // Fixed safety values
    // =============================================================

    static constexpr float _min_range{
        0.05f
    };


    static constexpr float _max_range{
        4.0f
    };


    static constexpr float _max_jump{
        0.30f
    };


    static constexpr float _max_vertical_speed{
        0.5f
    };


    static constexpr float _degraded_timeout_s{
        0.5f
    };


    static constexpr float _sensor_loss_timeout_s{
        2.0f
    };


    // =============================================================
    // PX4 parameters
    // =============================================================

    DEFINE_PARAMETERS(

        (ParamInt<px4::params::TOF_MODE>)
        _param_tof_mode,

        (ParamFloat<px4::params::TOF_ALT>)
        _param_tof_alt,

        (ParamFloat<px4::params::TOF_KP>)
        _param_tof_kp,

        (ParamFloat<px4::params::TOF_KD>)
        _param_tof_kd,

        (ParamFloat<px4::params::TOF_ALPHA>)
        _param_tof_alpha

    )
};


// =================================================================
// Global instance
// =================================================================

static ToFAltitudeControl *g_module{
    nullptr
};


// =================================================================
// PX4 module entry point
// =================================================================

extern "C"
__EXPORT int tof_altitude_control_main(
    int argc,
    char *argv[])
{
    if (argc < 2) {

        PX4_INFO(
            "Usage: tof_altitude_control "
            "{start|stop|status}"
        );


        return 1;
    }


    // =============================================================
    // START
    // =============================================================

    if (
        !strcmp(
            argv[1],
            "start"
        )
    ) {

        if (
            g_module
            != nullptr
        ) {

            PX4_WARN(
                "Already running"
            );


            return 1;
        }


        g_module =
            new ToFAltitudeControl();


        if (
            g_module
            == nullptr
        ) {

            PX4_ERR(
                "Allocation failed"
            );


            return 1;
        }


        const int task_id =
            px4_task_spawn_cmd(
                "tof_altitude_control",

                SCHED_DEFAULT,

                SCHED_PRIORITY_DEFAULT,

                3000,

                [](int task_argc,
                   char *task_argv[]) -> int {

                    (void)task_argc;
                    (void)task_argv;


                    const int ret =
                        g_module->run();


                    delete g_module;


                    g_module =
                        nullptr;


                    return ret;
                },

                nullptr
            );


        if (task_id < 0) {

            PX4_ERR(
                "Failed to start task"
            );


            delete g_module;


            g_module =
                nullptr;


            return 1;
        }


        return 0;
    }


    // =============================================================
    // STOP
    // =============================================================

    if (
        !strcmp(
            argv[1],
            "stop"
        )
    ) {

        if (
            g_module
            == nullptr
        ) {

            PX4_WARN(
                "Not running"
            );


            return 1;
        }


        g_module->request_stop();


        return 0;
    }


    // =============================================================
    // STATUS
    // =============================================================

    if (
        !strcmp(
            argv[1],
            "status"
        )
    ) {

        if (
            g_module
            != nullptr
        ) {

            PX4_INFO(
                "Running"
            );

        } else {

            PX4_INFO(
                "Stopped"
            );
        }


        return 0;
    }


    PX4_INFO(
        "Usage: tof_altitude_control "
        "{start|stop|status}"
    );


    return 1;
}
