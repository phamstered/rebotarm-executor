#!/usr/bin/env python3
"""Explicit bench-only ROS ACC transaction; never enables a motor or saves flash.

Run in the ROS Humble overlay. JSONL stdout is the durable evidence stream;
redirect it to a new file before APPLY. Observe is the default and creates no
publisher. APPLY/RESTORE require the current boot ID and an explicit action.
"""
import argparse
import json
import time

SOURCE = 0x4A3750524F46494C
REQUEST_TOPIC = '/stm32/joint7_bench_profile/request'
STATUS_TOPIC = '/stm32/joint7_bench_profile/status'
ACC_ORIGINAL = 0x40000000
DEC_ORIGINAL = 0xC0000000
ACC_TARGET = 0x3951B717


def decode_profile(words):
    if (len(words) != 32 or any(type(x) is not int or not 0 <= x <= 0xffffffff for x in words)
            or words[:3] != [0x4a375031, 1, 32]):
        raise ValueError('Invalid J7P1 status wire message')
    keys = ['phase', 'result', 'error', 'generation', 'source_lo', 'source_hi',
            'operation_id', 'action', 'profile_id', 'busy', 'dirty', 'original_valid',
            'original_acc', 'original_dec', 'current_acc', 'current_dec', 'read_mask',
            'applied_verified', 'restore_verified', 'started_ms', 'phase_ms',
            'completed_ms', 'last_reject_operation', 'last_reject_error',
            'write_attempts', 'rollback_attempts']
    out = dict(zip(keys, words[6:]))
    for key in ('busy', 'dirty', 'original_valid', 'applied_verified', 'restore_verified'):
        if out[key] not in (0, 1):
            raise ValueError('Invalid profile flag ' + key)
        out[key] = bool(out[key])
    if out['result'] > 6 or out['phase'] > 11 or out['read_mask'] > 3:
        raise ValueError('Unknown profile state')
    out['boot_id'] = str(words[3] | words[4] << 32)
    out['source_id'] = str(out.pop('source_lo') | out.pop('source_hi') << 32)
    out['mcu_ms'] = words[5]
    return out


def request_words(action, boot, control, profile):
    """Only fixed profile/action/identity, no caller-supplied register or value."""
    if action not in ('apply', 'restore') or boot <= 0 or boot >= 2**64:
        raise ValueError('Invalid explicit operation')
    if str(control['boot_id']) != str(boot) or profile['boot_id'] != str(boot):
        raise ValueError('Boot changed or disagrees with reviewed boot')
    if (not control['disabled_verified'] or control['mode'] != 3 or
            control['enabled_joint_mask'] or control['lease_id'] or control['switching']):
        raise ValueError('Fresh all-axis DISABLE proof and no lease/switch required')
    if profile['busy'] or profile['source_id'] not in ('0', str(SOURCE)):
        raise ValueError('Profile busy or owned by another source')
    if profile['operation_id'] >= 0xffffffff or profile['generation'] >= 0xffffffff:
        raise ValueError('Profile identity exhausted; never wrap automatically')
    if action == 'apply':
        if control['owner_source_id'] or control['fault_code']:
            raise ValueError('APPLY requires no owner and no fault')
        if control['pos_vel_supported_mask'] != 64:
            raise ValueError('APPLY requires joint7-only bench admission')
        if profile['result'] != 0 or profile['original_valid'] or profile['dirty']:
            raise ValueError('APPLY requires untouched profile; no automatic retry')
    elif (not profile['original_valid'] or not profile['dirty'] or
          profile['original_acc'] != ACC_ORIGINAL or profile['original_dec'] != DEC_ORIGINAL):
        raise ValueError('RESTORE requires same-boot recorded original bits and possible modification')
    operation = profile['operation_id'] + 1
    return [0x4a375131, 1, 14, boot & 0xffffffff, boot >> 32,
            SOURCE & 0xffffffff, SOURCE >> 32, operation, 1 if action == 'apply' else 2,
            1, control['control_epoch'], control['last_command_sequence'],
            profile['generation'], (profile['mcu_ms'] + 750) & 0xffffffff]


def evidence(profile, captured_at_ms):
    return dict(profile, schema='joint7_bench_profile_evidence_v1',
                captured_at_ms=captured_at_ms)


def validate_baseline(words, boot):
    if (len(words) != 87 or words[:3] != [0x4a375331, 1, 87]
            or str(words[3] | words[4] << 32) != str(boot)
            or any(words[i] for i in (6, 14, 15)) or words[10:13] != [37, 23, 11]
            or words[16:20] != [0x4a374233, 1, 0xfff, 64]
            or any(words[36:48]) or words[48] != 0
            or words[29:31] != [ACC_ORIGINAL, DEC_ORIGINAL]
            or words[53:56] != [1, 1, 0x1ff] or any(words[67:76])
            or words[58] != 0x41200000):  # The reviewed candidate assumes Gr=10.
        raise ValueError('Complete same-boot baseline with Gr=10 is required')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=['observe', 'apply', 'restore'], default='observe')
    parser.add_argument('--boot', type=int, required=True,
                        help='exact boot ID from the preceding reviewed preflight')
    args = parser.parse_args()
    if not 0 < args.boot < 2**64:
        parser.error('boot must be a nonzero uint64')

    # ROS imports stay local: protocol validation tests need no ROS installation.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import UInt32MultiArray
    from microros_stm32h723_bridge.msg import MotorControlStatus
    from rosidl_runtime_py.convert import message_to_ordereddict

    def emit(kind, value):
        print(json.dumps({'kind': kind, 'captured_at_ms': int(time.time()*1000),
                          'data': value}, allow_nan=False), flush=True)

    rclpy.init()
    node = rclpy.create_node('codex_joint7_bench_profile')
    qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    latest = {}
    subscriptions = []
    publisher = None

    def receive_control(msg):
        latest['control'] = (time.monotonic(), message_to_ordereddict(msg))

    def receive_profile(msg):
        if msg.layout.dim or msg.layout.data_offset:
            raise ValueError('Profile status requires empty layout')
        decoded = decode_profile(list(msg.data))
        latest['profile'] = (time.monotonic(), decoded)
        emit('profile', decoded)

    def receive_snapshot(msg):
        latest['snapshot'] = (time.monotonic(), list(msg.data))

    def endpoints(topic, subscription=False):
        infos = (node.get_subscriptions_info_by_topic(topic) if subscription else
                 node.get_publishers_info_by_topic(topic))
        return (len(infos) == 1 and infos[0].node_name == 'stm32h723_motor_bridge'
                and infos[0].node_namespace == '/')

    def fresh():
        now = time.monotonic()
        return all(key in latest and now-latest[key][0] <= .2 for key in ('control', 'profile'))

    try:
        subscriptions.append(node.create_subscription(MotorControlStatus,
            '/stm32/motor_control_status', receive_control, qos))
        subscriptions.append(node.create_subscription(UInt32MultiArray, STATUS_TOPIC, receive_profile, qos))
        subscriptions.append(node.create_subscription(UInt32MultiArray,
            '/stm32/joint7_bench_snapshot', receive_snapshot, qos))
        emit('start', {'action': args.action, 'expected_boot': str(args.boot),
                       'creates_motion_publishers': False, 'save_flash': False})
        end = time.monotonic() + 8
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if (fresh() and endpoints(STATUS_TOPIC) and endpoints('/stm32/motor_control_status')
                    and (args.action != 'apply' or
                         ('snapshot' in latest and endpoints('/stm32/joint7_bench_snapshot')))):
                break
        else:
            raise RuntimeError('Fresh MCU status/discovery unavailable; no request sent')
        control, profile = latest['control'][1], latest['profile'][1]
        if profile['boot_id'] != str(args.boot) or str(control['boot_id']) != str(args.boot):
            raise RuntimeError('Boot mismatch; no request sent')
        emit('preflight', {'control': control, 'profile': profile})
        if args.action == 'observe':
            if (not control['disabled_verified'] or control['mode'] != 3 or
                    control['enabled_joint_mask'] or control['lease_id'] or control['switching']):
                raise RuntimeError('Observation evidence requires current verified output-off')
            emit('evidence', evidence(profile, int(time.time()*1000)))
            return 0

        if args.action == 'apply':
            words = latest['snapshot'][1]
            if time.monotonic()-latest['snapshot'][0] > 1.5:
                raise RuntimeError('Startup baseline publication is stale; no request sent')
            validate_baseline(words, args.boot)
            emit('baseline_snapshot', words)
        publisher = node.create_publisher(UInt32MultiArray, REQUEST_TOPIC, qos)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if endpoints(REQUEST_TOPIC, subscription=True) and fresh():
                break
        else:
            raise RuntimeError('Exactly one MCU request subscriber required; no request sent')
        control, profile = latest['control'][1], latest['profile'][1]
        request = request_words(args.action, args.boot, control, profile)
        emit('request', request)  # Flush identity + original evidence before publishing.
        message = UInt32MultiArray()
        message.data = request
        sent_at = time.monotonic()
        for index in range(3):
            publisher.publish(message)  # Identical operation, never a new automatic attempt.
            until = time.monotonic() + .05
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.01)
        end = sent_at + 4
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            at, profile = latest['profile']
            if profile['boot_id'] != str(args.boot):
                raise RuntimeError('MCU boot changed; drive RAM restoration cannot be assumed')
            if at <= sent_at:
                continue
            if profile['last_reject_operation'] == request[7] and profile['last_reject_error']:
                raise RuntimeError('Profile request rejected: '+str(profile['last_reject_error']))
            if profile['source_id'] != str(SOURCE) or profile['operation_id'] != request[7] or profile['busy']:
                continue
            expected_result = 2 if args.action == 'apply' else 3
            flag = 'applied_verified' if args.action == 'apply' else 'restore_verified'
            acc = ACC_TARGET if args.action == 'apply' else ACC_ORIGINAL
            control = latest['control'][1]
            if (profile['result'] != expected_result or profile['error'] or not profile[flag]
                    or profile['read_mask'] != 3 or profile['current_acc'] != acc
                    or profile['current_dec'] != DEC_ORIGINAL
                    or profile['generation'] != request[12]+1
                    or profile['action'] != request[8] or profile['profile_id'] != 1
                    or not fresh() or str(control['boot_id']) != str(args.boot)
                    or not control['disabled_verified'] or control['mode'] != 3
                    or control['enabled_joint_mask'] or control['lease_id'] or control['switching']):
                emit('failed_terminal_state', profile)
                raise RuntimeError('Transaction did not independently verify requested state')
            emit('evidence', evidence(profile, int(time.time()*1000)))
            return 0
        raise RuntimeError('No terminal readback proof; keep output off and inspect state before recovery')
    except Exception as error:
        emit('error', str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
