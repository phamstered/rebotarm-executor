#include "motor_capabilities.h"
#include "motor_control_v2.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    MotorControlV2 s; motor_v2_init(&s, 42);
    s.owner_source_id=8; s.lease_id=9; s.selected_joint_mask=127;
    s.verified_joint_mask=127;
    MotorCommandV2 c={0}; c.boot_id=42; c.control_epoch=s.control_epoch;
    c.source_id=8; c.lease_id=9; c.active_joint_mask=127;
    c.sequence=2; c.mode=1; c.control_mode=2;
    for (unsigned i=0;i<7;i++) {
        const MotorConfiguredCapability *cap=&motor_configured_capabilities[i];
#if MOTOR_POSVEL_URDF_PROFILE
        assert(cap->pos_vel_enabled && cap->position_min < cap->position_max);
        assert(cap->velocity_limit_max == 0.1f);
#else
        assert(!cap->pos_vel_enabled && cap->position_min==0 && cap->position_max==0);
#endif
        if(cap->pos_vel_enabled) s.pos_vel_supported_mask |= 1U<<i;
        s.position_min[i]=cap->position_min; s.position_max[i]=cap->position_max;
        s.velocity_limit_max[i]=cap->velocity_limit_max; s.control_modes[i]=2;
        c.position[i]=(cap->position_min+cap->position_max)/2; c.velocity_limit[i]=0.1f;
    }
#if MOTOR_POSVEL_URDF_PROFILE
    assert(s.pos_vel_supported_mask==127);
    assert(motor_v2_command(&s,&c,0,true,1,10)==MV2_OK);
    for(unsigned i=0;i<7;i++) {
        MotorCommandV2 bad=c;
        bad.position[i]=nextafterf(s.position_min[i],-INFINITY);
        assert(motor_v2_command(&s,&bad,0,true,1,10)==MV2_LIMITS);
        bad.position[i]=nextafterf(s.position_max[i],INFINITY);
        assert(motor_v2_command(&s,&bad,0,true,1,10)==MV2_LIMITS);
        bad=c; bad.velocity_limit[i]=nextafterf(.1f,INFINITY);
        assert(motor_v2_command(&s,&bad,0,true,1,10)==MV2_LIMITS);
        bad=c; bad.position[i]=NAN;
        assert(motor_v2_command(&s,&bad,0,true,1,10)==MV2_NONFINITE);
    }
    assert(motor_v2_command(&s,&c,3,true,1,10)==MV2_FAULT);
    const double scale=-0.008612826752, offset=0.004282757541;
    assert(s.position_min[6]*scale+offset <= .0715);
    assert(s.position_max[6]*scale+offset >= 0);
    assert(fabs(s.position_min[6]*scale+offset-.0715)<1e-8);
    assert(fabs(s.position_max[6]*scale+offset)<1e-8);
    c.position[6]=.511749f; /* Current closed-end reading is NOT silently admitted. */
    assert(motor_v2_command(&s,&c,0,true,1,10)==MV2_LIMITS);
#else
    assert(s.pos_vel_supported_mask==0);
    assert(motor_v2_command(&s,&c,0,true,1,10)==MV2_LIMITS);
#endif
    puts("per-axis POS_VEL profile tests passed");
}
