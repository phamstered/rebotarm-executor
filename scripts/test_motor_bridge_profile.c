/* Actual adapter integration; all CAN traffic stays in this in-memory model. */
#include <assert.h>
#include <stdio.h>
#include "../User/motor_bridge.c"

FDCAN_HandleTypeDef hfdcan1;
HostRng host_rng={0,1,42,0};
static uint32_t clock_ms, drive_acc, drive_dec;
static BspFdcanFrame queue[256], history[2048];
static unsigned head, count, sent_count, writes, enables;
static bool stuck_tx, fail_feedback;
uint32_t HAL_GetTick(void) { return clock_ms; }
uint32_t fdcan1_tx_free(void) { return stuck_tx ? 0U : 8U; }
bool fdcan1_tx_empty(void) { return !stuck_tx; }

static void push(uint16_t id, const uint8_t data[8])
{
    assert(count<256);
    BspFdcanFrame *f=&queue[(head+count++)%256];
    f->id=id;f->received_ms=clock_ms;memcpy(f->data,data,8);
}
static void feedback(unsigned axis)
{
    uint8_t data[8]={0};data[0]=axis+1;
    uint16_t position=axis==6?0x851e:0x8000;
    data[1]=(uint8_t)(position>>8);data[2]=(uint8_t)position;
    data[3]=0x80;data[4]=0x08; /* near-zero quantized velocity and effort */
    push((uint16_t)(0x11+axis),data);
}
uint8_t fdcan1_get_rx_data_at(uint16_t *id,uint8_t *data,uint32_t *at)
{
    if(!count)return 0;
    BspFdcanFrame *f=&queue[head];head=(head+1)%256;--count;
    *id=f->id;memcpy(data,f->data,8);if(at)*at=f->received_ms;return 8;
}
uint8_t fdcan1_get_rx_data(uint16_t *id,uint8_t *data) { return fdcan1_get_rx_data_at(id,data,NULL); }
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *handle,uint16_t id,uint8_t *data,uint32_t len)
{
    (void)handle;assert(len==8 && sent_count<2048);
    if(stuck_tx)return 1;
    history[sent_count].id=id;memcpy(history[sent_count++].data,data,8);
    if(id!=0x7ff) {
        if(data[7]==0xfc)++enables;
        assert(data[7]==0xfd); /* This test never permits a motion/enable frame. */
        return 0;
    }
    unsigned axis=data[0]-1U;assert(axis<7 && data[1]==0);
    if(data[2]==DAMIAO_REG_FEEDBACK_REQ) {
        if(fail_feedback)return 1;
        feedback(axis);return 0;
    }
    assert(data[2]==0x33 || data[2]==0x55); /* no save, reset, clear, etc. */
    uint8_t answer[8];memcpy(answer,data,8);uint32_t value=0;
    if(data[2]==0x55) {
        assert(axis==6 && data[3]==4);
        memcpy(&value,data+4,4);
        assert(value==MOTOR_BENCH_PROFILE_ACC_TARGET || value==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
        drive_acc=value;++writes;
    } else if(data[3]==4)value=drive_acc;
    else if(data[3]==5)value=drive_dec;
    else if(data[3]==10)value=1;
    else {
        float f=data[3]==31?4.0f:data[3]==21?12.5f:data[3]==22?30.0f:data[3]==23?10.0f:1.0f;
        memcpy(&value,&f,4);
    }
    memcpy(answer+4,&value,4);push((uint16_t)(0x11+axis),answer);
    return 0;
}

static void fixture(void)
{
    memset(&state,0,sizeof(state));motor_v2_init(&state.control,42);
    motor_bench_profile_init(&motor_bench_profile,42);
    motor_bench_trace_init(&motor_bench_trace_meta,motor_bench_trace_records);
    memset((void*)&motor_bench_diagnostics,0,sizeof(motor_bench_diagnostics));
    state.initialized=true;state.snapshot.mode=MOTOR_MODE_DISABLE;
    state.snapshot.feedback_valid=true;state.feedback_valid_mask=0x7f;
    state.control.verified_joint_mask=0x7f;state.control.pos_vel_supported_mask=0x40;
    state.have_disable=state.have_command=true;state.disable_ms=50;state.snapshot.last_command_sequence=3;
    state.snapshot.position[6]=0.5f;
    for(unsigned i=0;i<7;++i) {
        state.control.control_modes[i]=1;state.drive_status[i]=0;
        state.received_feedback_ms[i]=state.snapshot.last_feedback_ms[i]=90;
        state.last_feedback_request_ms[i]=90;
        state.control.position_min[i]=MOTOR_BENCH_POSITION_MIN;
        state.control.position_max[i]=MOTOR_BENCH_POSITION_MAX;
        state.control.velocity_limit_max[i]=MOTOR_BENCH_VELOCITY_MAX;
    }
    head=count=sent_count=writes=enables=0;clock_ms=100;stuck_tx=fail_feedback=false;
    drive_acc=MOTOR_BENCH_PROFILE_ACC_ORIGINAL;drive_dec=MOTOR_BENCH_PROFILE_DEC_ORIGINAL;
}
static MotorBenchProfileRequest profile_request(uint32_t action)
{
    return (MotorBenchProfileRequest){.boot_id=state.control.boot_id,.source_id=123,
        .operation_id=motor_bench_profile.status.operation_id+1,.action=action,.profile_id=1,
        .expected_control_epoch=state.control.control_epoch,.disable_sequence=state.snapshot.last_command_sequence,
        .expected_profile_generation=motor_bench_profile.status.generation,.valid_until_ms=clock_ms+750};
}
static void tick(void) { motor_bridge_control_step(clock_ms);clock_ms+=4; }
static void run(void)
{
    unsigned limit=600;
    while(motor_bench_profile.status.busy && limit--)tick();
    assert(limit && !motor_bench_profile.status.busy && enables==0);
}
static void apply(void)
{
    MotorBenchProfileRequest r=profile_request(MBP_APPLY);
    assert(motor_bridge_bench_profile_request(&r,clock_ms));
}
static MotorBridgeCommand stop(uint32_t sequence)
{
    MotorBridgeCommand c={0};c.sequence=sequence;c.mode=MOTOR_MODE_DISABLE;return c;
}

int main(void)
{
    fixture();assert(disabled_proven(clock_ms));
    MotorControlRequest mode={.action=2,.operation_id=10,.boot_id=42,.expected_control_epoch=1,
        .source_id=5,.active_joint_mask=0x40,.control_mode=2,.disable_sequence=3};
    assert(motor_bridge_control_request(&mode,clock_ms).error_code==MV2_UNSUPPORTED);
    MotorCommandV2 move={.boot_id=42,.control_epoch=1,.source_id=5,.lease_id=5,.sequence=4,
        .mode=1,.control_mode=2,.active_joint_mask=0x40};move.position[6]=0.5f;move.velocity_limit[6]=0.02f;
    assert(!motor_bridge_accept_command_v2(&move,clock_ms) && state.control.reject_code==MV2_UNSUPPORTED);
    apply();
    MotorBridgeCommand legacy={.sequence=4,.mode=1};
    assert(!motor_bridge_accept_command(&legacy,clock_ms));
    assert(!motor_bridge_accept_command_v2(&move,clock_ms) && state.control.reject_code==MV2_BUSY);
    mode.action=1;assert(motor_bridge_control_request(&mode,clock_ms).error_code==MV2_BUSY);
    mode.action=2;assert(motor_bridge_control_request(&mode,clock_ms).error_code==MV2_BUSY);
    assert(!motor_bridge_request_zero_position(MOTOR_ZERO_OPERATION_GET,64));
    run();assert(motor_bench_profile_ready(&motor_bench_profile) && writes==1);
    assert(drive_acc==MOTOR_BENCH_PROFILE_ACC_TARGET && drive_dec==MOTOR_BENCH_PROFILE_DEC_ORIGINAL);

    /* Same-boot exact restoration works with the retained POS owner and fault. */
    state.control.owner_source_id=5;state.control.control_modes[6]=2;
    state.snapshot.fault_code=MOTOR_FAULT_COMMAND_LIMIT;motor_bench_diagnostics.trip_reason=4;
    MotorBenchProfileRequest r=profile_request(MBP_RESTORE);
    state.control.lease_id=8;assert(!motor_bridge_bench_profile_request(&r,clock_ms));state.control.lease_id=0;
    state.control.switching=true;assert(!motor_bridge_bench_profile_request(&r,clock_ms));state.control.switching=false;
    state.have_disable=false;assert(!motor_bridge_bench_profile_request(&r,clock_ms));state.have_disable=true;
    assert(motor_bridge_bench_profile_request(&r,clock_ms));run();
    assert(motor_bench_profile.status.result==MBP_RESTORED && drive_acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
    assert(state.snapshot.fault_code==MOTOR_FAULT_COMMAND_LIMIT && motor_bench_diagnostics.trip_reason==4);
    assert(state.control.owner_source_id==5 && state.control.control_modes[6]==2 && !state.control.lease_id);

    fixture();apply();while(!motor_bench_profile.status.dirty)tick();
    MotorBridgeCommand disable=stop(4);assert(motor_bridge_accept_command(&disable,clock_ms));
    run();assert(motor_bench_profile.status.result==MBP_ROLLED_BACK && drive_acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
    assert(writes==2 && !enables);

    /* STOP queue and failed feedback polling cannot starve the deadline. */
    for(unsigned failure=0;failure<2;++failure) {
        fixture();apply();while(!motor_bench_profile.status.dirty)tick();
        disable=stop(4);assert(motor_bridge_accept_command(&disable,clock_ms));
        if(!failure) { stuck_tx=true;state.disable_pending=false;state.disable_tx_wait=true; }
        else { fail_feedback=true;state.disable_pending=state.disable_tx_wait=false;
            for(unsigned i=0;i<7;++i)state.last_feedback_request_ms[i]=0; }
        run();assert(motor_bench_profile.status.result==MBP_RESTORE_REQUIRED);
        assert(motor_bench_profile_inhibits(&motor_bench_profile));
        assert(!motor_bridge_accept_command(&legacy,clock_ms));
        assert(motor_bridge_control_request(&mode,clock_ms).error_code==MV2_BUSY);
        assert(!motor_bridge_request_zero_position(MOTOR_ZERO_OPERATION_GET,64));
        disable=stop(5);assert(motor_bridge_accept_command(&disable,clock_ms));
        assert(!enables && motor_bench_profile.status.original_acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
    }

    /* Real initialization observes retained drive RAM but never invents a
     * previous boot's restoration journal or admits that nonbaseline profile. */
    fixture();drive_acc=MOTOR_BENCH_PROFILE_ACC_TARGET;motor_bridge_init();
    assert(state.initialized && !state.control.pos_vel_supported_mask);
    assert(!motor_bench_profile.status.original_valid && drive_acc==MOTOR_BENCH_PROFILE_ACC_TARGET);
    disable=stop(1);assert(motor_bridge_accept_command(&disable,clock_ms));
    tick();clock_ms+=4;for(unsigned i=0;i<7;++i)feedback(i);tick();
    assert(disabled_proven(clock_ms));r=profile_request(MBP_RESTORE);
    assert(!motor_bridge_bench_profile_request(&r,clock_ms));
    assert(motor_bench_profile.status.last_reject_error==MBP_NO_ORIGINAL && writes==0 && enables==0);
    puts("joint7 profile adapter gate and recovery tests passed");
}
