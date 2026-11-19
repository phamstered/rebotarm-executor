#ifndef MOTOR_BENCH_ROS_H
#define MOTOR_BENCH_ROS_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "motor_bench.h"

#if MOTOR_BENCH_JOINT7_POSVEL
#include "motor_bench_trace.h"
#include "motor_bench_startup_trace.h"
/* UInt32MultiArray.data contains numeric uint32 words, not native struct bytes.
 * There are no CAN/ROS/RTOS calls, allocation, or state mutations here.
 * TX_OK means HAL accepted an enqueue; it is NOT transmission completion. */
#define MOTOR_BENCH_ROS_VERSION 1U
#define MOTOR_BENCH_ROS_SNAPSHOT_MAGIC 0x4a375331U /* J7S1 */
#define MOTOR_BENCH_ROS_TRACE_MAGIC 0x4a375431U /* J7T1 */
#define MOTOR_BENCH_ROS_CALIBRATION_MAGIC 0x4a374331U /* J7C1 */
#define MOTOR_BENCH_ROS_STARTUP_MAGIC 0x4a374531U /* J7E1 */
#define MOTOR_BENCH_ROS_SNAPSHOT_HEADER_WORDS 16U
#define MOTOR_BENCH_ROS_ADMISSION_WORDS 37U
#define MOTOR_BENCH_ROS_LOOP_WORDS 23U
#define MOTOR_BENCH_ROS_META_WORDS 11U
#define MOTOR_BENCH_ROS_SNAPSHOT_WORDS 87U
#define MOTOR_BENCH_ROS_TRACE_HEADER_WORDS 24U
#define MOTOR_BENCH_ROS_RECORD_WORDS 4U
#define MOTOR_BENCH_ROS_TRACE_MAX_WORDS 56U
#define MOTOR_BENCH_ROS_CALIBRATION_HEADER_WORDS 8U
#define MOTOR_BENCH_ROS_CALIBRATION_WORDS 29U
#define MOTOR_BENCH_ROS_STARTUP_HEADER_WORDS 22U
#define MOTOR_BENCH_ROS_STARTUP_MAX_WORDS 54U

typedef struct {
    uint32_t free_heap_bytes;
    uint32_t min_ever_free_heap_bytes;
    uint32_t microros_stack_high_water_words;
    uint32_t can_rx_overflow_count;
} MotorBenchRuntimeDiagnostics;

_Static_assert(sizeof(float) == 4U, "bench float32 ABI");
_Static_assert(sizeof(MotorBenchDiagnostics) == 148U, "preserve J7B3 ABI");
_Static_assert(MOTOR_BENCH_LOOP_REGISTER_COUNT == 9U, "wire v1 loop RID count");
_Static_assert(sizeof(MotorBenchLoopDiagnostics) == 92U, "loop wire ABI");
_Static_assert(MOTOR_BENCH_CALIBRATION_REGISTER_COUNT == 8U, "wire v1 calibration RID count");
_Static_assert(sizeof(MotorBenchCalibrationDiagnostics) == 84U, "calibration wire ABI");

static inline void motor_bench_ros_meta(uint32_t *out, const MotorBenchTraceMeta *m)
{
    out[0] = m->version;
    out[1] = m->first_sequence;
    out[2] = m->next_sequence;
    out[3] = m->count;
    out[4] = m->overwritten;
    out[5] = m->armed;
    out[6] = m->triggered;
    out[7] = m->frozen;
    out[8] = m->trigger_kind;
    out[9] = m->trigger_ms;
    out[10] = m->tail_count;
}

/* Common words: magic, version, total words, boot low, boot high, MCU tick,
 * reserved=0. Snapshot words 7..9: runtime counters; 10..12: section lengths;
 * 13: cumulative CAN RX queue overflow count; 14..15 reserved=0.
 * Sections: unchanged J7B3, loop RID observations, trace meta.
 * The caller supplies one atomically copied snapshot and integer RTOS counters. */
static inline size_t motor_bench_ros_encode_snapshot(uint32_t *out, size_t capacity,
    uint64_t boot_id, uint32_t now_ms, const MotorBenchDiagnostics *admission,
    const MotorBenchLoopDiagnostics *loop, const MotorBenchTraceMeta *trace,
    const MotorBenchRuntimeDiagnostics *runtime)
{
    if (!out || !admission || !loop || !trace || !runtime ||
        capacity < MOTOR_BENCH_ROS_SNAPSHOT_WORDS) return 0U;
    memset(out, 0, MOTOR_BENCH_ROS_SNAPSHOT_WORDS * sizeof(*out));
    out[0] = MOTOR_BENCH_ROS_SNAPSHOT_MAGIC;
    out[1] = MOTOR_BENCH_ROS_VERSION;
    out[2] = MOTOR_BENCH_ROS_SNAPSHOT_WORDS;
    out[3] = (uint32_t)boot_id;
    out[4] = (uint32_t)(boot_id >> 32);
    out[5] = now_ms;
    out[7] = runtime->free_heap_bytes;
    out[8] = runtime->min_ever_free_heap_bytes;
    out[9] = runtime->microros_stack_high_water_words;
    out[10] = MOTOR_BENCH_ROS_ADMISSION_WORDS;
    out[11] = MOTOR_BENCH_ROS_LOOP_WORDS;
    out[12] = MOTOR_BENCH_ROS_META_WORDS;
    out[13] = runtime->can_rx_overflow_count;
    memcpy(out + 16U, admission, sizeof(*admission));
    memcpy(out + 53U, loop, sizeof(*loop));
    motor_bench_ros_meta(out + 76U, trace);
    return MOTOR_BENCH_ROS_SNAPSHOT_WORDS;
}

/* Separate startup observations preserve the historical snapshot/trace ABI.
 * Common words 0..6 as above; word 7 is reserved=0. Words 8..28 contain
 * version, complete, read mask, start/end ticks, raw[8], signed errors[8].
 * Completion means collection ended, not that reads or calibration succeeded. */
static inline size_t motor_bench_ros_encode_calibration(uint32_t *out, size_t capacity,
    uint64_t boot_id, uint32_t now_ms, const MotorBenchCalibrationDiagnostics *calibration)
{
    if (!out || !calibration || capacity < MOTOR_BENCH_ROS_CALIBRATION_WORDS) return 0U;
    memset(out, 0, MOTOR_BENCH_ROS_CALIBRATION_WORDS * sizeof(*out));
    out[0] = MOTOR_BENCH_ROS_CALIBRATION_MAGIC;
    out[1] = MOTOR_BENCH_ROS_VERSION;
    out[2] = MOTOR_BENCH_ROS_CALIBRATION_WORDS;
    out[3] = (uint32_t)boot_id;
    out[4] = (uint32_t)(boot_id >> 32);
    out[5] = now_ms;
    memcpy(out + MOTOR_BENCH_ROS_CALIBRATION_HEADER_WORDS, calibration, sizeof(*calibration));
    return MOTOR_BENCH_ROS_CALIBRATION_WORDS;
}

/* Trace words 7..17: metadata; 18: page start sequence; 19: page count;
 * 20: gap; 21: ring capacity; 22: record words; 23: reserved=0.
 * Records are [MCU tick, tag, data0, data1]. All lengths are words, not bytes. */
static inline size_t motor_bench_ros_encode_trace(uint32_t *out, size_t capacity,
    uint64_t boot_id, uint32_t now_ms, const MotorBenchTracePage *page,
    const MotorBenchTraceRecord *records)
{
    if (!out || !page || page->count > MOTOR_BENCH_TRACE_PAGE_MAX ||
        (page->count && !records)) return 0U;
    size_t words = MOTOR_BENCH_ROS_TRACE_HEADER_WORDS +
        page->count * MOTOR_BENCH_ROS_RECORD_WORDS;
    if (capacity < words) return 0U;
    memset(out, 0, words * sizeof(*out));
    out[0] = MOTOR_BENCH_ROS_TRACE_MAGIC;
    out[1] = MOTOR_BENCH_ROS_VERSION;
    out[2] = (uint32_t)words;
    out[3] = (uint32_t)boot_id;
    out[4] = (uint32_t)(boot_id >> 32);
    out[5] = now_ms;
    motor_bench_ros_meta(out + 7U, &page->meta);
    out[18] = page->first_sequence;
    out[19] = page->count;
    out[20] = page->gap;
    out[21] = MOTOR_BENCH_TRACE_CAPACITY;
    out[22] = MOTOR_BENCH_ROS_RECORD_WORDS;
    for (uint32_t i = 0U; i < page->count; ++i) {
        uint32_t *record = out + MOTOR_BENCH_ROS_TRACE_HEADER_WORDS + i * 4U;
        record[0] = records[i].ms;
        record[1] = records[i].tag;
        record[2] = records[i].data0;
        record[3] = records[i].data1;
    }
    return words;
}

/* Independent startup prefix: 7..15 metadata (version, armed, frozen, reason,
 * count, start tick, enable enqueued, enable tick, command sequence); 16 page
 * start, 17 page count, 18 gap, 19 capacity, 20 record words, 21 reserved=0.
 * This is a separate topic/magic; existing J7S1/J7T1/J7C1 remain unchanged. */
static inline size_t motor_bench_ros_encode_startup(uint32_t *out, size_t capacity,
    uint64_t boot_id, uint32_t now_ms, const MotorBenchStartupPage *page,
    const MotorBenchTraceRecord *records)
{
    if (!out || !page || page->count > MOTOR_BENCH_TRACE_PAGE_MAX ||
        (page->count && !records)) return 0U;
    size_t words = MOTOR_BENCH_ROS_STARTUP_HEADER_WORDS +
        page->count * MOTOR_BENCH_ROS_RECORD_WORDS;
    if (capacity < words) return 0U;
    memset(out, 0, words * sizeof(*out));
    out[0] = MOTOR_BENCH_ROS_STARTUP_MAGIC;
    out[1] = MOTOR_BENCH_ROS_VERSION;
    out[2] = (uint32_t)words;
    out[3] = (uint32_t)boot_id;
    out[4] = (uint32_t)(boot_id >> 32);
    out[5] = now_ms;
    const MotorBenchStartupMeta *m = &page->meta;
    out[7] = m->version;
    out[8] = m->armed;
    out[9] = m->frozen;
    out[10] = m->reason;
    out[11] = m->count;
    out[12] = m->start_ms;
    out[13] = m->enable_enqueued;
    out[14] = m->enable_ms;
    out[15] = m->command_sequence;
    out[16] = page->first_sequence;
    out[17] = page->count;
    out[18] = page->gap;
    out[19] = MOTOR_BENCH_STARTUP_CAPACITY;
    out[20] = MOTOR_BENCH_ROS_RECORD_WORDS;
    for (uint32_t i = 0U; i < page->count; ++i) {
        uint32_t *record = out + MOTOR_BENCH_ROS_STARTUP_HEADER_WORDS + i * 4U;
        record[0] = records[i].ms;
        record[1] = records[i].tag;
        record[2] = records[i].data0;
        record[3] = records[i].data1;
    }
    return words;
}
#endif
#endif
