/* Exercise the actual adapter and CAN encoding without RTOS, USB, or motors. */
#include <assert.h>
#include <stdio.h>
#include "../User/motor_bridge.c"
FDCAN_HandleTypeDef hfdcan1;
HostRng host_rng={0,1,42,0};
static uint32_t clock_ms;
static BspFdcanFrame sent[128], received[64];
static unsigned tx_count, rx_count, rx_pos;
static bool tx_empty = true;
static bool tick_advancing, fail_send;
bool fdcan1_tx_empty(void) { return tx_empty; }
uint32_t fdcan1_tx_free(void) { return 8U; }
uint32_t HAL_GetTick(void) { return tick_advancing ? clock_ms++ : clock_ms; }
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *h, uint16_t id, uint8_t *data, uint32_t len)
{
    (void)h; assert(tx_count<128 && len==8); sent[tx_count].id=id;
    memcpy(sent[tx_count++].data,data,8); return fail_send ? 1 : 0;
}
uint8_t fdcan1_get_rx_data_at(uint16_t *id, uint8_t *data, uint32_t *received_ms)
{
    if(rx_pos==rx_count) return 0;
    if (received_ms) *received_ms=received[rx_pos].received_ms;
    *id=received[rx_pos].id; memcpy(data,received[rx_pos++].data,8); return 8;
}
uint8_t fdcan1_get_rx_data(uint16_t *id, uint8_t *data) { return fdcan1_get_rx_data_at(id,data,0); }
static void feedback(unsigned i, bool enabled)
{
    BspFdcanFrame *f=&received[rx_count++]; memset(f,0,sizeof(*f));
    f->received_ms=clock_ms;
    f->id=feedback_ids[i]; f->data[0]=(uint8_t)((enabled?0x10:0)|motor_ids[i]);
    f->data[1]=0x80; f->data[3]=0x80; f->data[4]=0x08;
}
static void fixture(void)
{
    memset(&state,0,sizeof(state)); motor_v2_init(&state.control,42);
    state.initialized=true; state.snapshot.mode=3; state.snapshot.feedback_valid=true;
    state.feedback_valid_mask=MV2_ALL; state.control.verified_joint_mask=MV2_ALL;
    for(unsigned i=0;i<7;++i) { state.control.control_modes[i]=1; state.control.position_min[i]=-2; state.control.position_max[i]=2; state.control.velocity_limit_max[i]=1; state.snapshot.last_feedback_ms[i]=100; }
    state.control.pos_vel_supported_mask=1; clock_ms=100;tx_count=rx_count=rx_pos=0;
    tick_advancing=fail_send=false;
#if MOTOR_BENCH_JOINT7_POSVEL
    memset((void *)&motor_bench_diagnostics,0,sizeof(motor_bench_diagnostics));
#endif
}
static MotorBridgeCommand disable(uint32_t sequence)
{
    MotorBridgeCommand c={0}; c.sequence=sequence;c.mode=3;return c;
}
int main(void)
{
    fixture(); MotorBridgeCommand stop=disable(42);
    assert(motor_bridge_accept_command(&stop,100)); assert(!disabled_proven(100));
    for(unsigned i=0;i<7;++i) feedback(i,false);
    tx_empty=false; motor_bridge_control_step(101); assert(!disabled_proven(101));
    tx_empty=true; motor_bridge_control_step(102); assert(!disabled_proven(102));
    clock_ms=103; for(unsigned i=0;i<7;++i) feedback(i,false);
    motor_bridge_control_step(103); assert(disabled_proven(103));
    MotorControlRequest r={.action=1,.operation_id=1,.boot_id=42,.expected_control_epoch=1,.source_id=5,.active_joint_mask=1,.control_mode=2,.disable_sequence=42};
    assert(motor_bridge_control_request(&r,106).accepted);
    motor_bridge_control_step(107); assert(sent[tx_count-1].data[3]==DAMIAO_REG_CTRL_MODE);
    /* Readback must match CAN ID, motor ID, RID and current transaction phase. */
    BspFdcanFrame *f=&received[rx_count++]; memset(f,0,sizeof(*f));f->id=feedback_ids[0];
    damiao_encode_register_write_cmd(2,DAMIAO_REG_CTRL_MODE,2,f->data);
    motor_bridge_control_step(108); assert(state.control.switching && !state.control.control_modes[0]);
    f=&received[rx_count++];f->id=feedback_ids[0];damiao_encode_register_write_cmd(1,DAMIAO_REG_CTRL_MODE,2,f->data);
    motor_bridge_control_step(109); motor_bridge_control_step(129);
    f=&received[rx_count++];f->id=feedback_ids[0];damiao_encode_register_read_cmd(1,DAMIAO_REG_CTRL_MODE,f->data);f->data[4]=2;
    motor_bridge_control_step(130); assert(state.control.operation_state==MV2_SUCCESS);
    r.action=2;r.operation_id=2;r.expected_control_epoch=state.control.control_epoch;
    assert(motor_bridge_control_request(&r,130).accepted);
    MotorControlSnapshot status; clock_ms=130;
    motor_bridge_get_control_snapshot(&status); assert(status.disabled_verified);
    MotorCommandV2 c={.boot_id=42,.control_epoch=state.control.control_epoch,.source_id=5,.lease_id=state.control.lease_id,.sequence=43,.mode=1,.control_mode=2,.active_joint_mask=1};
    c.position[0]=0.4f;c.velocity_limit[0]=0.2f;
    assert(motor_bridge_accept_command_v2(&c,130));
    assert(!state.have_disable);
    motor_bridge_get_control_snapshot(&status); assert(!status.disabled_verified);
    MotorBridgeCommand legacy={.sequence=43,.mode=1};
    assert(!motor_bridge_accept_command(&legacy,130));assert(state.snapshot.fault_code==0);
    tx_count=0; motor_bridge_control_step(131);
    unsigned enables=0;
    for(unsigned i=0;i<tx_count;++i) if(sent[i].data[7]==0xfc) { assert(sent[i].id==0x101);++enables; }
    assert(enables==1);
    feedback(0,true);tx_count=0;motor_bridge_control_step(132);
    assert(tx_count==1 && sent[0].id==0x101);
    float pos,vel;memcpy(&pos,sent[0].data,4);memcpy(&vel,sent[0].data+4,4);
    assert(pos==0.4f && vel==0.2f);
    /* Stop acknowledges sequence, but enabled mask waits for physical feedback. */
    stop=disable(44);assert(motor_bridge_accept_command(&stop,133));
    tx_count=0;motor_bridge_control_step(134);
    bool stopped_pos=false;
    for(unsigned i=0;i<tx_count;++i) if(sent[i].id==0x101 && sent[i].data[7]==0xfd) stopped_pos=true;
    assert(stopped_pos);
    assert(state.motor_enabled[0]);assert(!state.control.lease_id);
    assert(!motor_bridge_accept_command_v2(&c,134));
    clock_ms=135; for(unsigned i=0;i<7;++i) feedback(i,false);
    motor_bridge_control_step(135);assert(!disabled_proven(135));
    clock_ms=136;for(unsigned i=0;i<7;++i) feedback(i,false);
    motor_bridge_control_step(136);assert(disabled_proven(136));
    /* Fault 11 remains latched, yet a new global DISABLE is always admitted. */
    state.communication_loss_latched=true;state.snapshot.fault_code=11;
    stop=disable(45);stop.position[0]=NAN;assert(motor_bridge_accept_command(&stop,136));
    assert(state.snapshot.fault_code==11 && state.command.position[0]==0);
    legacy.sequence=46;assert(!motor_bridge_accept_command(&legacy,136));
    fixture();
    state.control.control_modes[0]=2; state.control.owner_source_id=5;
    state.control.selected_joint_mask=1; state.control.lease_id=1; state.control.last_frame_ms=100;
    state.control.position_min[0]=1; state.control.position_max[0]=2;
    state.received_feedback_ms[0]=100; state.snapshot.position[0]=0.5f;
    c=(MotorCommandV2){.boot_id=42,.control_epoch=1,.source_id=5,.lease_id=1,.sequence=1,.mode=0,.control_mode=2,.active_joint_mask=1};
    c.velocity_limit[0]=0.2f;
    assert(!motor_bridge_accept_command_v2(&c,100)); assert(state.control.reject_code==MV2_LIMITS);
    state.snapshot.position[0]=1.5f;
    assert(motor_bridge_accept_command_v2(&c,100));
    tx_count=0; motor_bridge_control_step(101);
    bool primed=false;
    for(unsigned i=0;i<tx_count;++i) if(sent[i].id==0x101 && sent[i].data[7]!=0xfc) {
        memcpy(&pos,sent[i].data,4); assert(pos==1.5f); primed=true;
    }
    assert(primed); c.sequence=2;
    assert(!motor_bridge_accept_command_v2(&c,201)); assert(state.control.reject_code==MV2_FAULT);
#if MOTOR_BENCH_JOINT7_POSVEL
    fixture(); state.snapshot.position[6]=0.5f;
    /* Ignore a matching RID from the wrong CAN ID, write ACK and wrong motor. */
    for(unsigned j=0;j<3;++j) {
        f=&received[rx_count++];memset(f,0,sizeof(*f));f->id=j==0?0x11:0x17;
        damiao_encode_register_read_cmd(j==2?1:7,13,f->data);
        if(j==1) f->data[2]=DAMIAO_REG_WRITE_MARKER;
        f->data[4]=99;
    }
    for(unsigned j=0;j<MOTOR_BENCH_REGISTER_COUNT;++j) {
        f=&received[rx_count++];memset(f,0,sizeof(*f));f->id=0x17;
        uint8_t rid=motor_bench_registers[j];damiao_encode_register_read_cmd(7,rid,f->data);
        float value=rid==21?12.5f:rid==22?30.0f:rid==23?10.0f:1.0f;
        memcpy(f->data+4,&value,4);
    }
    initialize_joint7_bench();
    assert(motor_bench_diagnostics.complete && motor_bench_diagnostics.admitted_mask==0x40);
    assert(state.control.pos_vel_supported_mask==0x40 && state.control.position_min[0]==0);
    assert(state.control.position_min[6]==MOTOR_BENCH_POSITION_MIN);
    for(unsigned j=0;j<tx_count;++j) {
        assert(sent[j].id==damiao_get_register_can_id());
        assert(sent[j].data[0]==7 && sent[j].data[2]==DAMIAO_REG_READ_MARKER);
    }
    state.drive_status[0]=DAMIAO_STATUS_ENABLED;tx_count=0;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask && tx_count==0);
    fixture();state.snapshot.position[6]=0.5f;fail_send=true;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask);
    assert(motor_bench_diagnostics.read_mask==0 && motor_bench_diagnostics.errors[0]==DAMIAO_ERR_CAN_WRITE);
    fixture();state.snapshot.position[6]=0.5f;tick_advancing=true;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask);
    assert(motor_bench_diagnostics.read_mask==0 && motor_bench_diagnostics.errors[0]==DAMIAO_ERR_TIMEOUT);
    assert(clock_ms>=1100 && clock_ms<1600); /* Bounded even with no replies. */
    for(unsigned reason=1;reason<=5;++reason) {
        fixture();state.using_v2=true;state.have_command=true;state.snapshot.mode=1;
        state.command_v2.active_joint_mask=0x40;state.control.lease_id=4;
        state.control.control_modes[6]=2;state.control.pos_vel_supported_mask=0x40;
        state.snapshot.position[6]=0.5f;state.received_feedback_ms[6]=100;
        if(reason==1) state.received_feedback_ms[6]=0;
        if(reason==2) state.snapshot.velocity[6]=NAN;
        if(reason==3) state.snapshot.position[6]=0.6f;
        if(reason==4) state.snapshot.velocity[6]=0.2f;
        if(reason==5) state.snapshot.effort[6]=0.4f;
        motor_bridge_control_step(102);
        assert(state.snapshot.mode==3 && !state.control.lease_id && !state.control.pos_vel_supported_mask);
        assert(motor_bench_diagnostics.trip_reason==reason && tx_count==7);
        for(unsigned j=0;j<tx_count;++j) assert(sent[j].data[7]==0xfd);
        stop=disable(50);assert(motor_bridge_accept_command(&stop,103));
        assert(state.snapshot.fault_code!=0 && motor_bench_diagnostics.trip_reason==reason);
        legacy=(MotorBridgeCommand){.sequence=51,.mode=1};
        assert(!motor_bridge_accept_command(&legacy,104));
        state.snapshot.velocity[6]=state.snapshot.effort[6]=0;state.snapshot.position[6]=0.5f;
        motor_bridge_safety_step(105);assert(state.snapshot.mode==3 && state.snapshot.fault_code!=0);
        motor_bridge_control_step(106);
        clock_ms=107;for(unsigned j=0;j<7;++j) feedback(j,false);
        motor_bridge_control_step(108);assert(disabled_proven(108));
        uint32_t stopped_at=state.disable_ms;
        for(unsigned now=110;now<=160;now+=10) {
            motor_bridge_safety_step(now);motor_bridge_control_step(now);
            assert(disabled_proven(now) && state.disable_ms==stopped_at);
        }
    }
#endif
    puts("motor bridge V2 CAN tests passed");
}
