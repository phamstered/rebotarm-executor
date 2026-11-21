#ifndef MOTOR_CAPABILITIES_H
#define MOTOR_CAPABILITIES_H
#include <stdbool.h>
#ifndef MOTOR_POSVEL_URDF_PROFILE
#define MOTOR_POSVEL_URDF_PROFILE 0
#endif
#if MOTOR_POSVEL_URDF_PROFILE && MOTOR_BENCH_JOINT7_POSVEL
#error "Seven-axis configuration and joint7 bench image are mutually exclusive"
#endif
/* Explicit deployment configuration, not a claim of physical qualification.
 * joint1..6 use URDF radians directly. Joint7 uses raw motor radians:
 * finger_m = raw_rad * -0.008612826752 + 0.004282757541.
 * See docs/ros2-interface.md for source, units and calibration constraints. */
typedef struct {
    bool pos_vel_enabled;
    float position_min, position_max, velocity_limit_max;
} MotorConfiguredCapability;
static const MotorConfiguredCapability motor_configured_capabilities[7] = {
#if MOTOR_POSVEL_URDF_PROFILE
    {true, -2.8f, 2.8f, 0.1f},   /* joint1 DM4340P */
    {true, -3.14f, 0.0f, 0.1f},  /* joint2 DM4340P */
    {true, -3.14f, 0.0f, 0.1f},  /* joint3 DM4340P */
    {true, -1.87f, 1.57f, 0.1f}, /* joint4 DM4310 */
    {true, -1.57f, 1.57f, 0.1f}, /* joint5 DM4310 */
    {true, -3.14f, 3.14f, 0.1f}, /* joint6 DM4310 */
    /* Rounded inward at float precision, finger travel 0..0.0715 m. */
    {true, -7.8043180f, 0.4972534f, 0.1f}
#else
    {false, 0, 0, 0}, {false, 0, 0, 0}, {false, 0, 0, 0},
    {false, 0, 0, 0}, {false, 0, 0, 0}, {false, 0, 0, 0},
    {false, 0, 0, 0}
#endif
};
#endif
