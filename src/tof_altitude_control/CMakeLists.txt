px4_add_module(
    MODULE examples__tof_altitude_control
    MAIN tof_altitude_control

    SRCS
        tof_altitude_control.cpp
        tof_sensor.cpp

    MODULE_CONFIG
        module.yaml
)
