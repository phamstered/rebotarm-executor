/* Exercise the actual adapter and CAN encoding without RTOS, USB, or motors. */
#include <assert.h>
#include <stdio.h>
#include "../User/motor_bridge.c"
FDCAN_HandleTypeDef hfdcan1;
HostRng host_rng={0,1,42,0};
static uint32_t clock_ms;
static BspFdcanFrame sent[256], received[128];
static unsigned tx_count, rx_count, rx_pos;
static bool tx_empty = true;
static bool tick_advancing, fail_send;
bool fdcan1_tx_empty(void) { return tx_empty; }
uint32_t fdcan1_tx_free(void) { return 8U; }
uint32_t HAL_GetTick(void) { return tick_advancing ? clock_ms++ : clock_ms; }
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *h, uint16_t id, uint8_t *data, uint32_t len)
{
    (void)h; assert(tx_count<256 && len==8); sent[tx_count].id=id;
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
static void reg_reply(unsigned axis, uint8_t rid, bool write, uint32_t value, uint32_t at)
{
    BspFdcanFrame *f=&received[rx_count++];memset(f,0,sizeof(*f));
    f->id=feedback_ids[axis];f->received_ms=at;
    if(write) damiao_encode_register_write_cmd(motor_ids[axis],rid,value,f->data);
    else { damiao_encode_register_read_cmd(motor_ids[axis],rid,f->data);memcpy(f->data+4,&value,4); }
}
static void precise_reply(unsigned axis, float position, uint32_t at)
{
    uint32_t value;memcpy(&value,&position,4);
    reg_reply(axis,MV2_REG_PRECISE_POSITION,false,value,at);
}
static void fixture(void)
{
    memset(&state,0,sizeof(state)); motor_v2_init(&state.control,42);
    state.initialized=true; state.snapshot.mode=3; state.snapshot.feedback_valid=true;
    state.feedback_valid_mask=MV2_ALL; state.control.verified_joint_mask=MV2_ALL;
    for(unsigned i=0;i<7;++i) { state.control.control_modes[i]=1; state.control.position_min[i]=-2; state.control.position_max[i]=2; state.control.velocity_limit_max[i]=1; state.snapshot.last_feedback_ms[i]=100; state.received_feedback_ms[i]=100; }
    state.disable_ms=99;
    state.control.pos_vel_supported_mask=1; clock_ms=100;tx_count=rx_count=rx_pos=0;
    tick_advancing=fail_send=false;
#if MOTOR_BENCH_JOINT7_POSVEL
    memset((void *)&motor_bench_diagnostics,0,sizeof(motor_bench_diagnostics));
    memset((void *)&motor_bench_loop_diagnostics,0,sizeof(motor_bench_loop_diagnostics));
    motor_bench_init_calibration((MotorBenchCalibrationDiagnostics *)&motor_bench_calibration_diagnostics);
    motor_bench_trace_init(&motor_bench_trace_meta,motor_bench_trace_records);
    motor_bench_startup_init(&motor_bench_startup_meta,motor_bench_startup_records);
    motor_bench_profile_init(&motor_bench_profile,42);
    /* These existing adapter scenarios start with already-proven test profile. */
    motor_bench_profile.status.result=MBP_APPLIED;
    motor_bench_profile.status.applied_verified=1;
#endif
}
static MotorBridgeCommand disable(uint32_t sequence)
{
    MotorBridgeCommand c={0}; c.sequence=sequence;c.mode=3;return c;
}
static MotorCommandV2 begin_seed(bool hold)
{
    fixture();
    state.control.control_modes[0]=2;state.control.owner_source_id=5;
    state.control.selected_joint_mask=1;state.control.lease_id=1;
    state.control.last_frame_ms=100;state.snapshot.position[0]=0.125f;
    MotorCommandV2 c={.boot_id=42,.control_epoch=1,.source_id=5,.lease_id=1,
        .sequence=1,.mode=hold?0:1,.control_mode=2,.active_joint_mask=1};
    c.position[0]=hold?0:0.4f;c.velocity_limit[0]=0.02f;
    assert(motor_bridge_accept_command_v2(&c,100));
    motor_bridge_control_step(101);assert(state.seed_waiting && !state.seed_complete);
    return c;
}
static void no_enable(void)
{
    for(unsigned i=0;i<tx_count;++i) assert(sent[i].data[7]!=0xfc);
}
static void refresh_feedback_times(uint32_t now)
{
    for(unsigned i=0;i<7;++i) {
        state.received_feedback_ms[i]=now;state.snapshot.last_feedback_ms[i]=now;
    }
}
static void test_precise_seed_failures(void)
{
    for(unsigned variant=0;variant<13;++variant) {
        (void)begin_seed(true);no_enable();
        float position=variant==6?NAN:variant==7?3.0f:0.123456f;
        precise_reply(0,position,variant==5?100:102);
        BspFdcanFrame *f=&received[rx_count-1];
        if(variant==0) f->data[3]=0x51; /* Different position register. */
        if(variant==1) { f->id=feedback_ids[1];f->data[0]=2; }
        if(variant==2) f->data[0]=2;
        if(variant==3) f->id=0x77;
        if(variant==4) f->data[2]=DAMIAO_REG_WRITE_MARKER;
        if(variant==8) --rx_count; /* Lost reply, no blocking wait/retry. */
        if(variant==9) {
            MotorBridgeCommand stop=disable(2);
            assert(motor_bridge_accept_command(&stop,102));
        }
        if(variant==10) motor_v2_watchdog(&state.control,201);
        if(variant==11) state.snapshot.velocity[0]=0.021f;
        if(variant==12) fail_send=true;
        motor_bridge_control_step(variant==10?201:102);
        /* Send failure happens at precise prefill, before FC. */
        fail_send=false;no_enable();
        if(variant<6 || variant==8) {
            assert(state.seed_waiting);
            refresh_feedback_times(202);
            motor_bridge_control_step(202);
        }
        assert(state.snapshot.mode==MOTOR_MODE_DISABLE && !state.control.lease_id);
        no_enable();
        precise_reply(0,0.123456f,203);motor_bridge_control_step(203);no_enable();
    }
    (void)begin_seed(true);fail_send=true;
    /* A failed initial 0x50 enqueue is terminal, never retried. */
    state.seed_waiting=false;motor_bridge_control_step(102);fail_send=false;
    assert(state.snapshot.mode==MOTOR_MODE_DISABLE);no_enable();
}
static float last_target(void)
{
    for(unsigned i=tx_count;i>0;--i) {
        if(sent[i-1].id==0x101 && sent[i-1].data[7]!=0xfc && sent[i-1].data[7]!=0xfd) {
            float p;memcpy(&p,sent[i-1].data,4);return p;
        }
    }
    assert(false);return NAN;
}
static void test_precise_hold_and_watchdog(void)
{
    MotorCommandV2 c=begin_seed(true);
    precise_reply(0,0.123456f,102);motor_bridge_control_step(102);
    assert(state.seed_complete && state.hold_latched && state.hold_position[0]==0.123456f);
    /* Exactly one FC, even while waiting for physical enabled feedback. */
    tx_count=0;motor_bridge_control_step(122);no_enable();
    clock_ms=123;feedback(0,true);motor_bridge_control_step(123);
    assert(state.seed_enabled_seen==1);
    c.sequence=2;assert(motor_bridge_accept_command_v2(&c,150));
    tx_count=0;motor_bridge_control_step(151);
    float p=last_target();assert(p==0.123456f);
    assert(!state.seed_waiting); /* Repeated HOLD never reacquires a new seed. */
    refresh_feedback_times(251);motor_bridge_safety_step(251);
    assert(!state.control.lease_id && state.snapshot.mode==MOTOR_MODE_HOLD);
    tx_count=0;motor_bridge_control_step(251);no_enable();
    assert(state.snapshot.mode==MOTOR_MODE_HOLD && state.snapshot.fault_code==0);
    refresh_feedback_times(651);motor_bridge_safety_step(651);
    assert(state.snapshot.mode==MOTOR_MODE_HOLD && state.communication_loss_latched);
    tx_count=0;motor_bridge_control_step(651);no_enable();
    assert(state.snapshot.mode==MOTOR_MODE_HOLD && state.snapshot.fault_code==11);
    /* A seed acquired for TRACK must also survive a HOLD arriving before the
     * first enabled feedback; the explicit TRACK draft stays separate. */
    c=begin_seed(false);precise_reply(0,0.123456f,102);motor_bridge_control_step(102);
    c.sequence=2;c.mode=0;c.position[0]=0;
    assert(motor_bridge_accept_command_v2(&c,103));
    clock_ms=104;feedback(0,true);tx_count=0;motor_bridge_control_step(104);
    assert(state.hold_latched && state.hold_position[0]==0.123456f);
    p=last_target();assert(p==0.123456f);
    c.sequence=3;c.mode=1;c.position[0]=0.4f;
    assert(motor_bridge_accept_command_v2(&c,105));
    tx_count=0;motor_bridge_control_step(106);
    p=last_target();assert(p==0.4f);
    /* Lease loss while waiting for FC acknowledgment stops; it cannot revive
     * the pending enable from a late valid 0x50 response. */
    (void)begin_seed(false);precise_reply(0,0.123456f,102);motor_bridge_control_step(102);
    refresh_feedback_times(201);motor_v2_watchdog(&state.control,201);
    tx_count=0;motor_bridge_control_step(201);no_enable();
    assert(state.snapshot.mode==MOTOR_MODE_DISABLE);
}
int main(void)
{
    test_precise_seed_failures();
    test_precise_hold_and_watchdog();
    fixture(); MotorBridgeCommand stop=disable(42);
    assert(motor_bridge_accept_command(&stop,100)); assert(!disabled_proven(100));
    for(unsigned i=0;i<7;++i) feedback(i,false);
    tx_empty=false; motor_bridge_control_step(101); assert(!disabled_proven(101));
    tx_empty=true; motor_bridge_control_step(102); assert(!disabled_proven(102));
    clock_ms=103; for(unsigned i=0;i<7;++i) feedback(i,false);
    motor_bridge_control_step(103); assert(disabled_proven(103));
    MotorControlRequest r={.action=1,.operation_id=1,.boot_id=42,.expected_control_epoch=1,.source_id=5,.active_joint_mask=1,.control_mode=2,.disable_sequence=42};
    state.snapshot.velocity[0]=0.021f;
    assert(motor_bridge_control_request(&r,106).error_code==MV2_NOT_DISABLED);
    state.snapshot.velocity[0]=0;
    assert(motor_bridge_control_request(&r,106).accepted);
    motor_bridge_control_step(107); assert(sent[tx_count-1].data[3]==MV2_REG_PRECISE_POSITION);
    assert(state.control.control_modes[0]==1);
    precise_reply(0,0.125f,107);motor_bridge_control_step(107);
    assert(sent[tx_count-1].data[3]==DAMIAO_REG_CTRL_MODE);
    /* Readback must match CAN ID, motor ID, RID and current transaction phase. */
    BspFdcanFrame *f=&received[rx_count++]; memset(f,0,sizeof(*f));f->id=feedback_ids[0];f->received_ms=108;
    damiao_encode_register_write_cmd(2,DAMIAO_REG_CTRL_MODE,2,f->data);
    motor_bridge_control_step(108); assert(state.control.switching && !state.control.control_modes[0]);
    f=&received[rx_count++];f->id=feedback_ids[0];f->received_ms=109;damiao_encode_register_write_cmd(1,DAMIAO_REG_CTRL_MODE,2,f->data);
    motor_bridge_control_step(109); motor_bridge_control_step(129);
    f=&received[rx_count++];f->id=feedback_ids[0];f->received_ms=130;damiao_encode_register_read_cmd(1,DAMIAO_REG_CTRL_MODE,f->data);f->data[4]=2;
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
    assert(state.seed_waiting && !state.seed_complete);
    for(unsigned i=0;i<tx_count;++i) assert(sent[i].data[7]!=0xfc);
    precise_reply(0,0.123456f,131);tx_count=0;motor_bridge_control_step(131);
    unsigned enables=0;
    for(unsigned i=0;i<tx_count;++i) if(sent[i].data[7]==0xfc) { assert(sent[i].id==0x101);++enables; }
    assert(enables==1);
    float seed;memcpy(&seed,sent[0].data,4);assert(seed==0.123456f);
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
    assert(state.seed_waiting);
    precise_reply(0,1.500123f,102);motor_bridge_control_step(102);
    bool primed=false;
    for(unsigned i=0;i<tx_count;++i) if(sent[i].id==0x101 && sent[i].data[7]!=0xfc) {
        memcpy(&pos,sent[i].data,4); assert(pos==1.500123f); primed=true;
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
        float value=rid==4?2.0f:rid==5?-2.0f:rid==31?4.0f:rid==21?12.5f:rid==22?30.0f:rid==23?10.0f:1.0f;
        memcpy(f->data+4,&value,4);
    }
    for(unsigned j=0;j<MOTOR_BENCH_LOOP_REGISTER_COUNT;++j) {
        f=&received[rx_count++];memset(f,0,sizeof(*f));f->id=0x17;f->received_ms=80+j;
        damiao_encode_register_read_cmd(7,motor_bench_loop_registers[j],f->data);
        float value=j==0?10.0f:j==1?1000.0f:j==6?100.0f:0.0f;
        memcpy(f->data+4,&value,4);
    }
    const uint32_t calibration_bits[] = {UINT32_MAX,0x37e501f9U,14,0x7fc00001U,
        0x39551d0eU,0x3b900943U,0,0x3f800000U};
    for(unsigned j=0;j<MOTOR_BENCH_CALIBRATION_REGISTER_COUNT;++j) {
        f=&received[rx_count++];memset(f,0,sizeof(*f));f->id=0x17;
        damiao_encode_register_read_cmd(7,motor_bench_calibration_registers[j],f->data);
        memcpy(f->data+4,&calibration_bits[j],4);
    }
    initialize_joint7_bench();
    assert(motor_bench_diagnostics.complete && motor_bench_diagnostics.admitted_mask==0x40);
    assert(motor_bench_diagnostics.read_mask==0xfff && tx_count==29);
    assert(sent[11].data[3]==31 && motor_bench_float(motor_bench_diagnostics.raw[11])==4.0f);
    assert(motor_bench_loop_diagnostics.complete && motor_bench_loop_diagnostics.read_mask==0x1ff);
    assert(motor_bench_float(motor_bench_loop_diagnostics.raw[0])==10.0f);
    MotorBenchSnapshot bench_snapshot; motor_bridge_get_bench_snapshot(&bench_snapshot);
    assert(bench_snapshot.admission.magic==0x4a374233 && sizeof(bench_snapshot.admission)==148);
    assert(bench_snapshot.loop.raw[1]==motor_bench_loop_diagnostics.raw[1]);
    assert(bench_snapshot.calibration.version==1 && bench_snapshot.calibration.complete);
    assert(sizeof(bench_snapshot.calibration)==84 && bench_snapshot.calibration.read_mask==0xff);
    assert(!memcmp(bench_snapshot.calibration.raw,calibration_bits,sizeof(calibration_bits)));
    assert(bench_snapshot.calibration.started_ms>=bench_snapshot.loop.completed_ms);
    assert(bench_snapshot.calibration.completed_ms>=bench_snapshot.calibration.started_ms);
    for(unsigned j=0;j<MOTOR_BENCH_CALIBRATION_REGISTER_COUNT;++j)
        assert(sent[21+j].data[3]==motor_bench_calibration_registers[j]);
    MotorBenchTracePage trace_page;MotorBenchTraceRecord trace_records[8];
    uint32_t trace_cursor=0;bool exact_read_rx=false;
    while(motor_bridge_copy_bench_trace(trace_cursor,trace_records,8,&trace_page)) {
        for(unsigned j=0;j<trace_page.count;++j)
            if(trace_records[j].ms==80 && (trace_records[j].tag>>18)==MOTOR_BENCH_TRACE_RX)
                exact_read_rx=true;
        trace_cursor=trace_page.first_sequence+trace_page.count;
    }
    assert(exact_read_rx); /* Capture CAN receive time, not dequeue time. */
    assert(state.control.pos_vel_supported_mask==0x40 && state.control.position_min[0]==0);
    assert(state.control.position_min[6]==MOTOR_BENCH_POSITION_MIN);
    for(unsigned j=0;j<tx_count;++j) {
        assert(sent[j].id==damiao_get_register_can_id());
        assert(sent[j].data[0]==7 && sent[j].data[2]==DAMIAO_REG_READ_MARKER);
    }
    /* Reuse the valid first eleven replies: an unread Deta must still fail closed. */
    rx_pos=3;rx_count=3+11;tx_count=0;tick_advancing=true;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask);
    assert(motor_bench_diagnostics.read_mask==0x7ff);
    assert(motor_bench_diagnostics.errors[11]==DAMIAO_ERR_TIMEOUT);
    /* Extra loop reads failing never change the original admission outcome. */
    rx_pos=3;rx_count=3+12;tx_count=0;
    initialize_joint7_bench();assert(state.control.pos_vel_supported_mask==0x40);
    assert(!motor_bench_loop_diagnostics.read_mask && motor_bench_loop_diagnostics.errors[0]==DAMIAO_ERR_TIMEOUT);
    assert(motor_bench_calibration_diagnostics.complete && !motor_bench_calibration_diagnostics.read_mask);
    for(unsigned j=0;j<MOTOR_BENCH_CALIBRATION_REGISTER_COUNT;++j)
        assert(motor_bench_calibration_diagnostics.errors[j]==DAMIAO_ERR_TIMEOUT);
    state.drive_status[0]=DAMIAO_STATUS_ENABLED;tx_count=0;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask && tx_count==0);
    assert(motor_bench_calibration_diagnostics.complete && !motor_bench_calibration_diagnostics.read_mask);
    for(unsigned j=0;j<MOTOR_BENCH_CALIBRATION_REGISTER_COUNT;++j)
        assert(motor_bench_calibration_diagnostics.errors[j]==DAMIAO_ERR_NOT_DISABLED);
    fixture();state.snapshot.position[6]=0.5f;fail_send=true;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask);
    assert(motor_bench_diagnostics.read_mask==0 && motor_bench_diagnostics.errors[0]==DAMIAO_ERR_CAN_WRITE);
    assert(motor_bench_calibration_diagnostics.errors[0]==DAMIAO_ERR_CAN_WRITE);
    fixture();state.snapshot.position[6]=0.5f;tick_advancing=true;
    initialize_joint7_bench();assert(!state.control.pos_vel_supported_mask);
    assert(motor_bench_diagnostics.read_mask==0 && motor_bench_diagnostics.errors[0]==DAMIAO_ERR_TIMEOUT);
    assert(clock_ms>=3000 && clock_ms<3400); /* Twenty-nine bounded reads from 100 ms. */
    assert(motor_bench_calibration_diagnostics.errors[0]==DAMIAO_ERR_TIMEOUT);
    for(unsigned reason=1;reason<=5;++reason) {
        fixture();state.using_v2=true;state.have_command=true;state.snapshot.mode=1;
        state.command_v2.active_joint_mask=0x40;state.control.lease_id=4;
        state.control.control_modes[6]=2;state.control.pos_vel_supported_mask=0x40;
        state.snapshot.position[6]=0.5f;state.received_feedback_ms[6]=100;
        motor_bench_trace_meta.armed=1;state.snapshot.last_command_sequence=49;
        if(reason==1) state.received_feedback_ms[6]=0;
        if(reason==2) state.snapshot.velocity[6]=NAN;
        if(reason==3) state.snapshot.position[6]=0.6f;
        if(reason==4) state.snapshot.velocity[6]=0.2f;
        if(reason==5) state.snapshot.effort[6]=0.4f;
        clock_ms=102;motor_bridge_control_step(102);
        assert(state.snapshot.mode==3 && !state.control.lease_id && !state.control.pos_vel_supported_mask);
        assert(motor_bench_diagnostics.trip_reason==reason && tx_count==7);
        assert(motor_bridge_copy_bench_trace(0,trace_records,8,&trace_page)>0);
        assert((trace_records[0].tag>>18)==MOTOR_BENCH_TRACE_TRIP);
        assert(trace_records[0].data0==49 && trace_records[0].data1==reason);
        assert(trace_page.meta.trigger_kind==MOTOR_BENCH_TRACE_TRIP && trace_page.meta.trigger_ms==102);
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
    /* The actual adapter records only joint7, arms on admitted V2, then keeps
     * the first stop evidence. Publishing/copying cannot consume or reset it. */
    fixture();
    state.control.control_modes[6]=2;state.control.owner_source_id=5;
    state.control.lease_id=7;state.control.selected_joint_mask=0x40;
    state.control.pos_vel_supported_mask=0x40;
    state.control.position_min[6]=MOTOR_BENCH_POSITION_MIN;
    state.control.position_max[6]=MOTOR_BENCH_POSITION_MAX;
    state.control.velocity_limit_max[6]=MOTOR_BENCH_VELOCITY_MAX;
    state.snapshot.position[6]=0.5f;state.received_feedback_ms[6]=100;
    stop=disable(7);assert(motor_bridge_accept_command(&stop,100));
    motor_bridge_get_bench_snapshot(&bench_snapshot);
    assert(!bench_snapshot.trace.armed && !bench_snapshot.trace.triggered);
    state.control.lease_id=7;
    c=(MotorCommandV2){.boot_id=42,.control_epoch=1,.source_id=5,.lease_id=7,
        .sequence=8,.mode=1,.control_mode=2,.active_joint_mask=0x40};
    c.position[6]=0.6f;c.velocity_limit[6]=0.02f;
    assert(!motor_bridge_accept_command_v2(&c,100));
    motor_bridge_get_bench_snapshot(&bench_snapshot);assert(!bench_snapshot.trace.armed);
    c.position[6]=0.5f;assert(motor_bridge_accept_command_v2(&c,100));
    /* The preflight's pending STOP must complete before the next control step. */
    state.disable_pending=state.disable_tx_wait=false;
    state.disable_ms=99; /* fixture models newly received post-stop feedback */
    tx_count=0;motor_bridge_control_step(100);
    precise_reply(6,0.5f,100);motor_bridge_control_step(100);
    MotorBenchStartupPage startup_page;MotorBenchTraceRecord startup_records[8];
    assert(motor_bridge_copy_bench_startup_trace(0,startup_records,8,&startup_page)==6);
    assert(startup_page.meta.armed && !startup_page.meta.frozen && startup_page.meta.enable_enqueued);
    assert((startup_records[0].tag>>18)==MOTOR_BENCH_STARTUP_START_KIND);
    assert((startup_records[1].tag>>18)==MOTOR_BENCH_TRACE_ACCEPTED_V2);
    assert((startup_records[2].tag>>18)==MOTOR_BENCH_TRACE_TX_OK &&
        (startup_records[2].tag&0x7ff)==0x7ff && (startup_records[2].data0>>24)==0x50);
    assert((startup_records[3].tag>>18)==MOTOR_BENCH_TRACE_RX &&
        (startup_records[3].data0>>24)==0x50);
    assert((startup_records[4].tag&0x7ff)==0x107 && startup_records[4].data0==0x3f000000);
    assert(startup_records[5].data0==UINT32_MAX && startup_records[5].data1==0xfcffffff);
    const uint8_t expected_posvel[]={0,0,0,0x3f,0x0a,0xd7,0xa3,0x3c};
    f=&received[rx_count++];memset(f,0,sizeof(*f));f->id=0x17;f->received_ms=98;
    const uint8_t raw_feedback[]={0x17,0x85,0x1e,0x80,0x08,0,0,0};
    memcpy(f->data,raw_feedback,8);clock_ms=104;motor_bridge_control_step(104);
    fail_send=true;assert(!send_frame(0x107,(uint8_t *)expected_posvel));fail_send=false;
    bool saw_accepted=false,saw_tx=false,saw_rx=false,saw_tx_error=false;
    trace_cursor=0;
    while(motor_bridge_copy_bench_trace(trace_cursor,trace_records,8,&trace_page)) {
        for(unsigned j=0;j<trace_page.count;++j) {
            MotorBenchTraceRecord *record=&trace_records[j];
            unsigned kind=record->tag>>18;
            assert(((record->tag>>15)&7)==7 && (record->tag&0xffc00000U)==0);
            if(kind==MOTOR_BENCH_TRACE_ACCEPTED_V2) {
                assert(record->data0==8 && record->data1==(1U|(2U<<8)|(64U<<16)));saw_accepted=true;
            }
            if(kind==MOTOR_BENCH_TRACE_TX_OK && (record->tag&0x7ff)==0x107 &&
                !memcmp(&record->data0,expected_posvel,8)) saw_tx=true;
            if(kind==MOTOR_BENCH_TRACE_RX && record->ms==98) {
                assert(record->ms==98 && !memcmp(&record->data0,raw_feedback,8));saw_rx=true;
            }
            if(kind==MOTOR_BENCH_TRACE_TX_ERROR) saw_tx_error=true;
        }
        trace_cursor=trace_page.first_sequence+trace_page.count;
    }
    assert(saw_accepted && saw_tx && saw_rx && saw_tx_error);
    clock_ms=105;stop=disable(9);assert(motor_bridge_accept_command(&stop,105));
    motor_bridge_get_bench_snapshot(&bench_snapshot);
    assert(bench_snapshot.trace.armed && bench_snapshot.trace.triggered && !bench_snapshot.trace.frozen);
    assert(bench_snapshot.trace.trigger_kind==MOTOR_BENCH_TRACE_STOP && bench_snapshot.trace.trigger_ms==105);
    motor_bridge_copy_bench_startup_trace(0,startup_records,8,&startup_page);
    assert(startup_page.meta.frozen && startup_page.meta.reason==MOTOR_BENCH_STARTUP_STOP);
    uint32_t startup_count=startup_page.meta.count;
    clock_ms=106;motor_bridge_control_step(106);
    clock_ms=204;motor_bridge_control_step(204);
    motor_bridge_get_bench_snapshot(&bench_snapshot);assert(!bench_snapshot.trace.frozen);
    clock_ms=205;motor_bridge_control_step(205);
    motor_bridge_get_bench_snapshot(&bench_snapshot);assert(bench_snapshot.trace.frozen);
    uint32_t frozen_next=bench_snapshot.trace.next_sequence;
    clock_ms=206;stop=disable(10);assert(motor_bridge_accept_command(&stop,206));
    motor_bridge_control_step(206);motor_bridge_get_bench_snapshot(&bench_snapshot);
    assert(bench_snapshot.trace.next_sequence==frozen_next && bench_snapshot.trace.trigger_ms==105);
    motor_bridge_copy_bench_startup_trace(0,startup_records,8,&startup_page);
    assert(startup_page.meta.count==startup_count && startup_page.meta.command_sequence==8);
#endif
    puts("motor bridge V2 CAN tests passed");
}
