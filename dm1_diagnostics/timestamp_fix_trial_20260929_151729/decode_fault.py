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



cal = {
    ("can0", "004"): ("FR_hip", +1), ("can0", "005"): ("FR_thigh", -1),
    ("can0", "006"): ("FR_calf", -1), ("can0", "001"): ("FL_hip", +1),
    ("can0", "002"): ("FL_thigh", +1), ("can0", "003"): ("FL_calf", +1),
    ("can1", "004"): ("RR_hip", -1), ("can1", "005"): ("RR_thigh", -1),
    ("can1", "006"): ("RR_calf", -1), ("can1", "001"): ("RL_hip", -1),
    ("can1", "002"): ("RL_thigh", +1), ("can1", "003"): ("RL_calf", +1),
}
# First enabled->disabled transition after the gait starts.
transitions=[]
for key, items in feedback.items():
    previous=None
    for stamp, decoded in items:
        status=decoded[0]
        if previous == 1 and status == 0 and stamp > 1790666320.0:
            transitions.append((stamp,key))
            break
        previous=status
fault_time,key=min(transitions)
print(f"first_disable={fault_time:.6f} key={key}")
print("last aligned command/feedback before first disable:")
for motor_key,(name,direction) in cal.items():
    cmd_items=[x for x in commands.get(motor_key,[]) if x[0] <= fault_time]
    fb_items=feedback.get(motor_key,[])
    fb_items=[x for x in fb_items if x[0] <= fault_time]
    if not cmd_items or not fb_items: continue
    ct,(cq_raw,cdq_raw,kp,kd,ff_raw)=cmd_items[-1]
    ft,(status,fq_raw,fdq_raw,ftau_raw,mos,rotor)=fb_items[-1]
    cq,cdq,ff=direction*cq_raw,direction*cdq_raw,direction*ff_raw
    fq,fdq,ftau=direction*fq_raw,direction*fdq_raw,direction*ftau_raw
    total=kp*(cq-fq)+kd*(cdq-fdq)+ff
    print(f"{name:9s} status={status:X} cmd_age={(fault_time-ct)*1000:6.1f}ms fb_age={(fault_time-ft)*1000:6.1f}ms q={fq:+.3f}->{cq:+.3f} dq={fdq:+.2f}->{cdq:+.2f} kp={kp:5.1f} kd={kd:.2f} ff={ff:+6.2f} pd={total-ff:+7.2f} total={total:+7.2f} fb_tau={ftau:+6.2f}")
print("\nFR/RL thigh commands in final 0.8s:")
for motor_key in [("can0","005"),("can1","002")]:
    name,direction=cal[motor_key]
    print(name)
    last_bucket=None
    for stamp,decoded in commands[motor_key]:
        if fault_time-0.8 <= stamp <= fault_time:
            bucket=int((stamp-(fault_time-0.8))*20)
            if bucket==last_bucket: continue
            last_bucket=bucket
            q,dq,kp,kd,ff=decoded
            print(f" {(stamp-fault_time)*1000:7.1f}ms q={direction*q:+.3f} dq={direction*dq:+.2f} kp={kp:.1f} kd={kd:.2f} ff={direction*ff:+.2f}")
