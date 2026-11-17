#include "motor_control_v2.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned writes, reads;
static bool bus_ok = true;
static bool send_reg(uint8_t axis, bool write, uint32_t value, void *context)
{
    (void)context; assert(axis < 7); assert(value == 1 || value == 2);
    if (write) ++writes; else ++reads;
    return bus_ok;
}
static MotorControlV2 fixture(void)
{
    MotorControlV2 s; motor_v2_init(&s, 0x1234567887654321ULL);
    s.verified_joint_mask = MV2_ALL;
    for (unsigned i=0;i<7;++i) { s.control_modes[i]=1; s.position_min[i]=-2; s.position_max[i]=2; s.velocity_limit_max[i]=1; }
    return s;
}
static MotorControlRequest request(const MotorControlV2 *s, uint8_t action, uint32_t op, uint32_t mask)
{
    MotorControlRequest r = {0}; r.action=action; r.operation_id=op; r.boot_id=s->boot_id;
    r.expected_control_epoch=s->control_epoch; r.source_id=99; r.active_joint_mask=mask;
    r.control_mode=2; r.disable_sequence=42; return r;
}
static MotorCommandV2 command(const MotorControlV2 *s, uint32_t sequence)
{
    MotorCommandV2 c = {0}; c.boot_id=s->boot_id; c.control_epoch=s->control_epoch;
    c.source_id=s->owner_source_id; c.lease_id=s->lease_id; c.sequence=sequence; c.mode=1;
    c.control_mode=2; c.active_joint_mask=s->selected_joint_mask;
    for(unsigned i=0;i<7;++i) if(c.active_joint_mask&(1U<<i)) { c.position[i]=0.5f; c.velocity_limit[i]=0.3f; }
    return c;
}
static uint32_t complete_config(MotorControlV2 *s, uint32_t now)
{
    while(s->switching) {
        uint8_t axis=s->axis; motor_v2_step(s,true,now,send_reg,0);
        assert(!(s->verified_joint_mask&(1U<<axis)));
        motor_v2_register_reply(s,(axis+1)%7,true,s->target_mode,now); assert(s->switching);
        motor_v2_register_reply(s,axis,true,s->target_mode,now);
        motor_v2_step(s,true,now+20,send_reg,0);
        motor_v2_register_reply(s,axis,false,s->target_mode,now+21); now+=24;
    }
    assert(s->operation_state==MV2_SUCCESS); return now;
}
int main(void)
{
    MotorControlV2 s=fixture(); assert(motor_v2_mit_allowed(&s));
    MotorControlRequest r=request(&s,1,1,5);
    assert(motor_v2_request(&s,&r,true,0,42,0).error_code==MV2_UNSUPPORTED);
    s.pos_vel_supported_mask=5;
    assert(motor_v2_request(&s,&r,false,0,42,0).error_code==MV2_NOT_DISABLED);
    assert(motor_v2_request(&s,&r,true,0,43,0).error_code==MV2_NOT_DISABLED);
    assert(motor_v2_request(&s,&r,true,11,42,0).error_code==MV2_FAULT);
    MotorControlRequest bad=r; bad.boot_id++; assert(motor_v2_request(&s,&bad,true,0,42,0).error_code==MV2_IDENTITY);
    assert(motor_v2_request(&s,&r,true,0,42,0).accepted); assert(s.switching);
    assert(!motor_v2_mit_allowed(&s));
    assert(motor_v2_request(&s,&r,true,0,42,0).accepted); assert(writes==0);
    bad=r; bad.active_joint_mask=1; assert(motor_v2_request(&s,&bad,true,0,42,0).error_code==MV2_INVALID);
    uint32_t now=complete_config(&s,0); assert(writes==2 && reads==2); assert(s.control_epoch==2);
    assert(s.control_modes[0]==2 && s.control_modes[1]==1 && s.control_modes[2]==2);
    r=request(&s,2,2,5); assert(motor_v2_request(&s,&r,true,0,42,now).accepted);
    uint32_t lease=s.lease_id; assert(lease);
    assert(s.operation_id==2 && s.operation_state==MV2_SUCCESS);
    assert(motor_v2_request(&s,&r,true,0,42,now).lease_id==lease);
    bad=request(&s,1,3,5);
    assert(motor_v2_request(&s,&bad,true,0,42,now).error_code==MV2_BUSY);
    MotorCommandV2 c=command(&s,43);
    assert(motor_v2_command(&s,&c,0,true,42,now)==0);
    assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_SEQUENCE);
    c=command(&s,44); c.source_id++; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_LEASE);
    c=command(&s,44); c.lease_id++; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_LEASE);
    c=command(&s,44); c.control_epoch++; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_IDENTITY);
    c=command(&s,44); c.mode=2; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_MODE);
    c=command(&s,44); c.active_joint_mask=1; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_INVALID);
    c=command(&s,44); c.position[1]=1; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_INVALID);
    c=command(&s,44); c.velocity_limit[0]=NAN; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_NONFINITE);
    c=command(&s,44); c.velocity_limit[0]=0; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_LIMITS);
    c=command(&s,44); c.position[0]=3; assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_LIMITS);
    c=command(&s,44); assert(motor_v2_command(&s,&c,11,true,43,now)==MV2_FAULT);
    assert(motor_v2_command(&s,&c,0,true,43,now)==0 && s.reject_code==0);
    c.mode=0;
    assert(motor_v2_command(&s,&c,0,true,43,now)==MV2_INVALID);
    c.position[0]=c.position[2]=0;
    s.position_min[0]=s.position_min[2]=1;
    assert(motor_v2_command(&s,&c,0,true,43,now)==0); /* actual HOLD feedback checked by adapter */
    s.position_min[0]=s.position_min[2]=-2;
    c.sequence=0; assert(motor_v2_command(&s,&c,0,true,UINT32_MAX,now)==0);
    motor_v2_watchdog(&s,now+101); assert(!s.lease_id);
    assert(!motor_v2_request(&s,&r,true,0,42,now+101).accepted);
    assert(motor_v2_command(&s,&c,0,true,0,now+101)==MV2_LEASE);
    /* New owner cannot acquire a stale lease; explicit disabled configure can take over. */
    r=request(&s,2,3,5); r.source_id=100;
    assert(motor_v2_request(&s,&r,true,0,42,now+102).error_code==MV2_BUSY);
    r.action=1; r.control_mode=1; r.active_joint_mask=MV2_ALL;
    assert(motor_v2_request(&s,&r,false,0,42,now+102).error_code==MV2_BUSY);
    assert(motor_v2_request(&s,&r,true,0,42,now+102).accepted);
    now=complete_config(&s,now+102); assert(!motor_v2_mit_allowed(&s));
    r=request(&s,3,4,MV2_ALL); r.source_id=100; r.control_mode=1;
    assert(motor_v2_request(&s,&r,true,0,42,now).accepted); assert(motor_v2_mit_allowed(&s));
    assert(s.operation_id==4 && s.operation_state==MV2_SUCCESS);
    /* Cancellation, failed writes, wrong readback and timeout never claim verification. */
    for(unsigned variant=0;variant<5;++variant) {
        s=fixture(); s.pos_vel_supported_mask=1; r=request(&s,1,1,1);
        assert(motor_v2_request(&s,&r,true,0,42,0).accepted);
        if(variant==0) { motor_v2_disable(&s); assert(s.operation_state==MV2_CANCELLED); }
        else {
            bus_ok=variant!=1; motor_v2_step(&s,true,0,send_reg,0); bus_ok=true;
            if(variant==2) motor_v2_register_reply(&s,0,true,1,1);
            if(variant==3) motor_v2_step(&s,true,101,send_reg,0);
            if(variant==4) motor_v2_step(&s,false,1,send_reg,0);
            assert(s.operation_state==MV2_FAILURE && !(s.verified_joint_mask&1));
        }
        assert(!s.switching && !s.lease_id && s.control_epoch==2);
        assert(!motor_v2_mit_allowed(&s));
    }
    /* Only an unused, disabled grant gets a 500 ms handshake window. */
    s=fixture(); s.pos_vel_supported_mask=1; s.control_modes[0]=2;
    r=request(&s,2,1,1); assert(motor_v2_request(&s,&r,true,0,42,0).accepted);
    motor_v2_watchdog(&s,499); assert(s.lease_id && !s.lease_has_frame);
    c=command(&s,43); assert(motor_v2_command(&s,&c,0,true,42,499)==0);
    assert(s.lease_has_frame); motor_v2_watchdog(&s,599); assert(s.lease_id);
    motor_v2_watchdog(&s,600); assert(!s.lease_id);
    s=fixture(); s.pos_vel_supported_mask=1; s.control_modes[0]=2;
    r=request(&s,2,1,1); assert(motor_v2_request(&s,&r,true,0,42,0).accepted);
    c=command(&s,43); assert(motor_v2_command(&s,&c,0,true,42,501)==MV2_LEASE);
    motor_v2_watchdog(&s,501); assert(!s.lease_id);
    assert(!motor_v2_request(&s,&r,true,0,42,501).accepted);
    s=fixture(); s.boot_id=0; r=request(&s,1,1,1);
    assert(motor_v2_request(&s,&r,true,0,42,0).error_code==MV2_IDENTITY);
    puts("motor control V2 tests passed");
}
