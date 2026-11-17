#ifndef MOTOR_CAPABILITIES_H
#define MOTOR_CAPABILITIES_H
#include <stdbool.h>
/* Fill an entry only after recording a model/firmware bench qualification in
 * docs/ros2-interface.md. MIT encoding ranges are not POS_VEL safety limits. */
typedef struct {
    bool pos_vel_qualified;
    float position_min, position_max, velocity_limit_max;
} MotorQualifiedCapability;
static const MotorQualifiedCapability motor_qualified_capabilities[7] = {
    {false, 0, 0, 0}, /* joint1 DM4340P */
    {false, 0, 0, 0}, /* joint2 DM4340P */
    {false, 0, 0, 0}, /* joint3 DM4340P */
    {false, 0, 0, 0}, /* joint4 DM4310 */
    {false, 0, 0, 0}, /* joint5 DM4310 */
    {false, 0, 0, 0}, /* joint6 DM4310 */
    {false, 0, 0, 0}  /* gripper DM4310 */
};
#endif
