import bisect
import re

LOG = "can.log"
POS_MAX = 12.566
VEL_MAX = 20.0
TAU_MAX = 120.0
KP_MAX = 500.0
KD_MAX = 5.0


def u2f(value, low, high, bits):
    return value / ((1 << bits) - 1) * (high - low) + low


def decode_command(payload):
    b = bytes.fromhex(payload)
    q = (b[0] << 8) | b[1]
    dq = (b[2] << 4) | (b[3] >> 4)
    kp = ((b[3] & 0x0F) << 8) | b[4]
    kd = (b[5] << 4) | (b[6] >> 4)
    tau = ((b[6] & 0x0F) << 8) | b[7]
    return (
        u2f(q, -POS_MAX, POS_MAX, 16),
        u2f(dq, -VEL_MAX, VEL_MAX, 12),
        u2f(kp, 0.0, KP_MAX, 12),
        u2f(kd, 0.0, KD_MAX, 12),
        u2f(tau, -TAU_MAX, TAU_MAX, 12),
    )


def decode_feedback(payload):
    b = bytes.fromhex(payload)
    q = (b[1] << 8) | b[2]
    dq = (b[3] << 4) | (b[4] >> 4)
    tau = ((b[4] & 0x0F) << 8) | b[5]
    return (
        b[0] >> 4,
        u2f(q, -POS_MAX, POS_MAX, 16),
        u2f(dq, -VEL_MAX, VEL_MAX, 12),
        u2f(tau, -TAU_MAX, TAU_MAX, 12),
        b[6],
        b[7],
    )


pattern = re.compile(r"\((\d+\.\d+)\) (can[01]) ([0-9A-F]{3})#([0-9A-F]{16})")
commands = {}
feedback = {}
statuses = []
master_to_motor = {"011": "001", "012": "002", "013": "003", "014": "004", "015": "005", "016": "006"}
for line in open(LOG, encoding="ascii"):
    match = pattern.match(line)
    if not match:
        continue
    stamp, bus, can_id, data = float(match[1]), match[2], match[3], match[4]
    if can_id in {"001", "002", "003", "004", "005", "006"} and not data.startswith("FFFFFFFF"):
        commands.setdefault((bus, can_id), []).append((stamp, decode_command(data)))
    if can_id in master_to_motor:
        motor_id = master_to_motor[can_id]
        decoded = decode_feedback(data)
        feedback.setdefault((bus, motor_id), []).append((stamp, decoded))
        if decoded[0] != 0:
            statuses.append((stamp, bus, motor_id, decoded[0]))

motors = {
    ("can0", "005"): ("FR_thigh", -1),
    ("can0", "006"): ("FR_calf", -1),
    ("can1", "002"): ("RL_thigh", 1),
    ("can1", "003"): ("RL_calf", 1),
}


def latest_before(items, stamp):
    stamps = [item[0] for item in items]
    index = bisect.bisect_right(stamps, stamp) - 1
    return items[index] if index >= 0 else None


print("nonzero status counts:")
counts = {}
for _, bus, motor_id, status in statuses:
    counts[(bus, motor_id, status)] = counts.get((bus, motor_id, status), 0) + 1
for key, count in sorted(counts.items()):
    print(key, count)

print("\nlogical command/feedback samples (absolute second minus 1790665600):")
for relative_second in range(33, 57):
    stamp = 1790665600.0 + relative_second
    print(f"t={relative_second:02d}s", end="")
    for key, (name, direction) in motors.items():
        command = latest_before(commands[key], stamp)
        state = latest_before(feedback[key], stamp)
        if command is None or state is None:
            continue
        command_age = stamp - command[0]
        state_age = stamp - state[0]
        q_des, dq_des, kp, kd, tau_ff = command[1]
        status, q, dq, tau, _, _ = state[1]
        q_des *= direction
        dq_des *= direction
        tau_ff *= direction
        q *= direction
        dq *= direction
        tau *= direction
        total = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
        print(
            f" | {name} st={status:X} age={max(command_age, state_age)*1000:4.0f}ms"
            f" q={q:+.2f}->{q_des:+.2f} dq={dq:+.2f}->{dq_des:+.2f}"
            f" ff={tau_ff:+.1f} est={total:+.1f}",
            end="",
        )
    print()

print("\ncommand gaps over 20 ms while enabled:")
reference_commands = commands[("can0", "004")]
for (previous_stamp, _), (stamp, _) in zip(reference_commands, reference_commands[1:]):
    gap = stamp - previous_stamp
    if 1790665604.0 <= previous_stamp <= 1790665656.3 and gap > 0.020:
        print(
            f"from={previous_stamp - 1790665600.0:.6f}s"
            f" to={stamp - 1790665600.0:.6f}s gap={gap:.6f}s"
        )

print("\nfirst status 1 -> 0 transition per motor:")
for key, items in sorted(feedback.items()):
    previous_status = None
    for stamp, decoded in items:
        status = decoded[0]
        if previous_status == 1 and status == 0:
            print(key, f"t={stamp - 1790665600.0:.6f}s")
            break
        previous_status = status
