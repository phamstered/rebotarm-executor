#include <assert.h>
#include <stdio.h>
#include "motor_bench_profile.h"

static MotorBenchProfile p;
static MotorBenchProfileEnvironment env;
static uint32_t now, acc, dec, sends, writes, backpressure_until;
static bool fail_read, fail_write, drop_ack, drop_read, corrupt_after_write;
static uint8_t pending[8];
static bool have_reply;

static void fixture(void)
{
    motor_bench_profile_init(&p,42);
    env=(MotorBenchProfileEnvironment){.boot_id=42,.control_epoch=7,.disable_sequence=30,
        .initialized=true,.disabled_verified=true,.owner_free=true,.lease_free=true,
        .not_switching=true,.fault_free=true,.zero_idle=true};
    now=100;acc=MOTOR_BENCH_PROFILE_ACC_ORIGINAL;dec=MOTOR_BENCH_PROFILE_DEC_ORIGINAL;
    sends=writes=backpressure_until=0;
    fail_read=fail_write=drop_ack=drop_read=corrupt_after_write=have_reply=false;
}

static MotorBenchProfileRequest request(uint32_t action)
{
    return (MotorBenchProfileRequest){.boot_id=42,.source_id=123,.operation_id=p.status.operation_id+1,
        .action=action,.profile_id=1,.expected_control_epoch=env.control_epoch,
        .disable_sequence=env.disable_sequence,.expected_profile_generation=p.status.generation,
        .valid_until_ms=now+750};
}

static int send_register(uint8_t rid, bool write, uint32_t value, void *context)
{
    (void)context;
    assert(rid==4 || rid==5);
    if (now<backpressure_until) return 1;
    ++sends;
    assert(!have_reply);
    if (write) {
        /* No DEC, persistence, mode, zero or enable operation is expressible. */
        assert(rid==4 && (value==MOTOR_BENCH_PROFILE_ACC_TARGET || value==MOTOR_BENCH_PROFILE_ACC_ORIGINAL));
        ++writes;
        if (fail_write) return -1;
        acc=value;
        if (corrupt_after_write && value==MOTOR_BENCH_PROFILE_ACC_TARGET) acc=0x3f800000;
        if (drop_ack) return 0;
    } else if (fail_read) return -1;
    else if (drop_read) return 0;
    memset(pending,0,8);pending[0]=7;pending[2]=write?0x55:0x33;pending[3]=rid;
    uint32_t answer=write?value:rid==4?acc:dec;
    memcpy(pending+4,&answer,4);have_reply=true;
    return 0;
}

static void tick(void)
{
    motor_bench_profile_step(&p,&env,now,send_register,NULL);
    if (have_reply) {
        have_reply=false;
        motor_bench_profile_reply(&p,0x17,pending,now,now);
    }
    now+=4;
}

static void run(void)
{
    unsigned limit=1000;
    while(p.status.busy && limit--) tick();
    assert(!p.status.busy && limit);
}

static void admit_apply(void)
{
    MotorBenchProfileRequest r=request(MBP_APPLY);
    assert(motor_bench_profile_request(&p,&r,&env,now));
}

int main(void)
{
    fixture();
    uint32_t input[14]={MOTOR_BENCH_PROFILE_REQUEST_MAGIC,1,14,42,0,123,0,1,1,1,7,30,0,850};
    MotorBenchProfileRequest r;
    assert(!motor_bench_profile_decode_request(input,13,&r));
    input[0]=0;assert(!motor_bench_profile_decode_request(input,14,&r));input[0]=MOTOR_BENCH_PROFILE_REQUEST_MAGIC;
    assert(motor_bench_profile_decode_request(input,14,&r));
    assert(r.boot_id==42 && r.source_id==123 && r.disable_sequence==30);
    assert(motor_bench_profile_request(&p,&r,&env,now));
    assert(motor_bench_profile_request(&p,&r,&env,now+2000)); /* Identical replay does no work. */
    ++r.valid_until_ms;assert(!motor_bench_profile_request(&p,&r,&env,now));
    assert(p.status.last_reject_error==MBP_INVALID);
    run();
    assert(p.status.result==MBP_APPLIED && motor_bench_profile_ready(&p));
    assert(!motor_bench_profile_inhibits(&p) && p.status.dirty && p.status.read_mask==3);
    assert(acc==MOTOR_BENCH_PROFILE_ACC_TARGET && dec==MOTOR_BENCH_PROFILE_DEC_ORIGINAL);
    assert(p.status.original_acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL && p.status.original_valid);
    assert(sends==5 && writes==1 && p.status.write_attempts==1);
    uint32_t out[33];memset(out,0xa5,sizeof(out));
    assert(!motor_bench_profile_encode_status(out,31,&p.status,now));
    assert(out[0]==0xa5a5a5a5);
    assert(motor_bench_profile_encode_status(out,32,&p.status,now)==32 && out[32]==0xa5a5a5a5);
    assert(out[7]==MBP_APPLIED && out[18]==MOTOR_BENCH_PROFILE_ACC_ORIGINAL && out[23]==1);
    r=request(MBP_APPLY);assert(!motor_bench_profile_request(&p,&r,&env,now)); /* Original immutable. */
    assert(p.status.last_reject_error==MBP_BUSY);
    env.owner_free=env.fault_free=false;env.control_epoch=9;env.disable_sequence=33;
    r=request(MBP_RESTORE);assert(motor_bench_profile_request(&p,&r,&env,now));
    run();assert(p.status.result==MBP_RESTORED && p.status.restore_verified && !p.status.dirty);
    assert(!env.owner_free && !env.fault_free); /* Restoration never clears either. */
    assert(acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL && dec==MOTOR_BENCH_PROFILE_DEC_ORIGINAL);
    assert(!motor_bench_profile_ready(&p));

    /* Every admission gate refuses without sending anything. */
    for(unsigned failure=0;failure<17;++failure) {
        fixture();r=request(MBP_APPLY);
        switch(failure) {
            case 0:r.boot_id=43;break;case 1:r.source_id=0;break;
            case 2:r.expected_control_epoch++;break;case 3:r.disable_sequence++;break;
            case 4:r.expected_profile_generation++;break;case 5:r.valid_until_ms=now;break;
            case 6:r.valid_until_ms=now+1001;break;case 7:r.action=3;break;
            case 8:r.profile_id=2;break;case 9:env.initialized=false;break;
            case 10:env.disabled_verified=false;break;case 11:env.owner_free=false;break;
            case 12:env.lease_free=false;break;case 13:env.not_switching=false;break;
            case 14:env.fault_free=false;break;case 15:env.zero_idle=false;break;
            case 16:r.action=MBP_RESTORE;break;
        }
        assert(!motor_bench_profile_request(&p,&r,&env,now) && !sends && !p.status.busy);
    }
    for(unsigned bad=0;bad<2;++bad) {
        fixture();if(bad)dec=0xbf800000;else acc=MOTOR_BENCH_PROFILE_ACC_TARGET;
        admit_apply();run();assert(p.status.result==MBP_FAILED && p.status.error==MBP_BASELINE);
        assert(writes==0 && !p.status.original_valid);
    }
    fixture();drop_ack=true;admit_apply();run();
    assert(p.status.result==MBP_APPLIED && p.status.read_mask==3 && writes==1);
    r=request(MBP_RESTORE);assert(motor_bench_profile_request(&p,&r,&env,now));run();
    assert(p.status.result==MBP_RESTORED && p.status.read_mask==3 && writes==2);

    fixture();corrupt_after_write=true;admit_apply();run();
    assert(p.status.result==MBP_ROLLED_BACK && p.status.error==MBP_READBACK);
    assert(p.status.restore_verified && !p.status.dirty && acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
    assert(p.status.rollback_attempts==1 && writes==2);
    fixture();fail_write=true;admit_apply();run();
    assert(p.status.result==MBP_RESTORE_REQUIRED && p.status.dirty && motor_bench_profile_inhibits(&p));
    assert(p.status.error==MBP_CAN && writes==2); /* A failed send is conservatively dirty. */
    fail_write=false;r=request(MBP_RESTORE);assert(motor_bench_profile_request(&p,&r,&env,now));run();
    assert(p.status.result==MBP_RESTORED && !p.status.dirty);

    fixture();backpressure_until=now+12;admit_apply();run();assert(p.status.result==MBP_APPLIED);
    fixture();backpressure_until=now+1000;admit_apply();run();
    assert(p.status.result==MBP_FAILED && p.status.error==MBP_TIMEOUT && sends==0);
    fixture();fail_read=true;admit_apply();run();assert(p.status.result==MBP_FAILED && !writes);
    fixture();drop_read=true;admit_apply();run();assert(p.status.result==MBP_FAILED && p.status.error==MBP_TIMEOUT && !writes);

    /* A queued old response, wrong source/header or future timestamp is ignored. */
    fixture();drop_read=true;admit_apply();tick();
    uint32_t phase_ms=p.status.phase_ms;
    uint8_t reply[8]={7,0,0x33,4,0,0,0,0};memcpy(reply+4,&acc,4);
    motor_bench_profile_reply(&p,0x11,reply,phase_ms,now);assert(p.status.phase==MBP_WAIT_ACC);
    reply[0]=6;motor_bench_profile_reply(&p,0x17,reply,phase_ms,now);reply[0]=7;
    reply[2]=0x55;motor_bench_profile_reply(&p,0x17,reply,phase_ms,now);reply[2]=0x33;
    reply[3]=5;motor_bench_profile_reply(&p,0x17,reply,phase_ms,now);reply[3]=4;
    motor_bench_profile_reply(&p,0x17,reply,phase_ms-1,now);
    motor_bench_profile_reply(&p,0x17,reply,now+1,now);assert(p.status.phase==MBP_WAIT_ACC);
    motor_bench_profile_reply(&p,0x17,reply,phase_ms,now);assert(p.status.phase==MBP_READ_DEC);

    fixture();admit_apply();motor_bench_profile_cancel(&p,now);
    assert(p.status.result==MBP_FAILED && p.status.error==MBP_CANCELLED && !writes);
    fixture();admit_apply();tick();tick();tick();assert(p.status.dirty);
    motor_bench_profile_cancel(&p,now);uint32_t recovery_at=p.recovery_started_ms;
    motor_bench_profile_cancel(&p,now+10);assert(p.recovery_started_ms==recovery_at);
    run();assert(p.status.result==MBP_ROLLED_BACK && acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL);
    fixture();admit_apply();tick();tick();tick();motor_bench_profile_cancel(&p,now);
    env.disabled_verified=false;
    run();assert(p.status.result==MBP_RESTORE_REQUIRED && p.status.dirty);

    fixture();admit_apply();run();dec=0xbf800000;env.fault_free=false;
    r=request(MBP_RESTORE);assert(motor_bench_profile_request(&p,&r,&env,now));run();
    assert(acc==MOTOR_BENCH_PROFILE_ACC_ORIGINAL && dec==0xbf800000);
    assert(p.status.result==MBP_RESTORE_REQUIRED && motor_bench_profile_inhibits(&p));
    fixture();now=UINT32_MAX-20;r=request(MBP_APPLY);
    assert(motor_bench_profile_request(&p,&r,&env,now));run();assert(p.status.result==MBP_APPLIED);
    /* A new MCU boot has no original journal, even if drive RAM retained ACC. */
    motor_bench_profile_init(&p,43);env.boot_id=43;r=request(MBP_RESTORE);r.boot_id=43;
    assert(!motor_bench_profile_request(&p,&r,&env,now) && p.status.last_reject_error==MBP_NO_ORIGINAL);
    puts("joint7 volatile profile transaction tests passed");
}
