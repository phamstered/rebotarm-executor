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
bool fdcan1_tx_empty(void) { return tx_empty; }
uint32_t HAL_GetTick(void) { return clock_ms; }
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *h, uint16_t id, uint8_t *data, uint32_t len)
{
    (void)h; assert(tx_count<128 && len==8); sent[tx_count].id=id;
    memcpy(sent[tx_count++].data,data,8); return 0;
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
    tx_count=0;motor_bridge_control_step(134);assert(sent[0].id==0x101 && sent[0].data[7]==0xfd);
    assert(state.motor_enabled[0]);assert(!state.control.lease_id);
    assert(!motor_bridge_accept_command_v2(&c,134));
    clock_ms=135; for(unsigned i=0;i<7;++i) feedback(i,false);
    motor_bridge_control_step(135);assert(disabled_proven(135));
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
    puts("motor bridge V2 CAN tests passed");
}
