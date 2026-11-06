#!/usr/bin/env python3
"""
用静止站立的诊断 CSV 标定 IMU 安装偏差（roll/pitch）.

原理：足端在机身系的坐标只由编码器经运动学算出，与 IMU 无关。四脚着地时，
足端平面的法向就是“地面竖直方向在机身系中的表示”；IMU 姿态给出的机身系竖直方向
与它的差 = 安装偏差 + 地面坡度。同一位置原地转 180° 再站一次，地面坡度在机身系中
反号，两次平均即可消去，得到纯安装偏差。只给一个文件时假设地面水平。

小角度下 roll/pitch 可直接相加：新安装角 = 采集时使用的安装角 + 平均误差。
yaw 安装角无法由重力观测，保持不变。

用法：
  imu_mount_calibration.py [--current-mount ROLL,PITCH] CSV [CSV ...]
CSV 来自 hardware_main --keyboard-control（dm1_diagnostics/real_tests/dm1_march_*.csv），
期间按 1 站起后静止站立 10 s 以上。
"""

import argparse
import csv
import math
import statistics
import sys

LEGS = ('FR', 'FL', 'RR', 'RL')
# 键盘按 1 站起后保持在 STAND_UP，不一定进入 STANCE，两者都算静止站立。
STANCE_STATES = {'OPENDOG_STAND_UP', 'OPENDOG_STANCE'}
SETTLE_S = 4.0          # 进入站立状态后丢弃的起身过渡时间，s。
MAX_GYRO = 0.03         # 静止判据：|角速度| 上限，rad/s。
MIN_FRAMES = 250        # 每个文件至少需要的静止帧数（500 Hz 下 0.5 s）。


def tilt_of_up(vx, vy, vz):
    """把机身系中的竖直单位向量换算为机身 roll/pitch（度，ZYX 约定）."""
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    vx, vy, vz = vx / norm, vy / norm, vz / norm
    roll = math.atan2(vy, vz)
    pitch = math.asin(max(-1.0, min(1.0, -vx)))
    return math.degrees(roll), math.degrees(pitch)


def foot_plane_up(points):
    """最小二乘拟合 z = a*x + b*y + c，返回指向机身一侧的平面法向."""
    n = len(points)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sz = sum(p[2] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    syy = sum(p[1] * p[1] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    sxz = sum(p[0] * p[2] for p in points)
    syz = sum(p[1] * p[2] for p in points)
    # 3x3 正规方程，用克拉默法则求解。
    m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    rhs = [sxz, syz, sz]

    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    coeffs = []
    for col in range(2):
        mc = [row[:] for row in m]
        for r in range(3):
            mc[r][col] = rhs[r]
        coeffs.append(det(mc) / d)
    a, b = coeffs
    return -a, -b, 1.0


def imu_up(qw, qx, qy, qz):
    """R_wb^T * e_z：IMU 姿态给出的世界竖直方向在机身系中的表示."""
    return (2.0 * (qx * qz - qw * qy),
            2.0 * (qy * qz + qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy))


def analyse(path):
    """返回 (imu 倾角, 足端平面倾角, 扭曲 cm, 帧数)，均为静止站立帧的中位数."""
    imu_roll, imu_pitch, foot_roll, foot_pitch, twist = [], [], [], [], []
    stance_start = None
    with open(path, newline='') as handle:
        for row in csv.DictReader(handle):
            t = float(row['t_s'])
            if row['fsm'] not in STANCE_STATES:
                stance_start = None
                continue
            if stance_start is None:
                stance_start = t
            if t - stance_start < SETTLE_S:
                continue
            if any(row.get('opendog_%s_contact' % leg, '1') not in ('1', '1.0', 'true')
                   for leg in LEGS):
                continue
            gyro = math.sqrt(sum(float(row['imu_g' + a]) ** 2 for a in 'xyz'))
            if gyro > MAX_GYRO:
                continue
            feet = [tuple(float(row['%s_foot_body_%s' % (leg, a)]) for a in 'xyz')
                    for leg in LEGS]
            r, p = tilt_of_up(*imu_up(*(float(row['imu_q' + c]) for c in 'wxyz')))
            fr, fp = tilt_of_up(*foot_plane_up(feet))
            z = {leg: foot[2] for leg, foot in zip(LEGS, feet)}
            imu_roll.append(r)
            imu_pitch.append(p)
            foot_roll.append(fr)
            foot_pitch.append(fp)
            twist.append(100.0 * ((z['FL'] + z['RR']) - (z['FR'] + z['RL'])))
    count = len(imu_roll)
    if count < MIN_FRAMES:
        return None, count
    med = statistics.median
    return (med(imu_roll), med(imu_pitch), med(foot_roll), med(foot_pitch),
            med(twist)), count


def main():
    """解析参数、逐文件分析并打印建议安装角."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--current-mount', default='0,0',
                        help='采集时生效的安装角 ROLL,PITCH（度），HFI 默认 0,0')
    parser.add_argument('csv', nargs='+')
    args = parser.parse_args()
    mount0 = [float(v) for v in args.current_mount.split(',')]

    errors = []
    for path in args.csv:
        result, count = analyse(path)
        if result is None:
            print('%s: 静止四脚站立帧只有 %d 帧，跳过' % (path, count))
            continue
        ir, ip, fr, fp, tw = result
        er, ep = ir - fr, ip - fp
        errors.append((er, ep))
        print('%s: %d 帧\n  IMU 机身倾角 roll=%+.2f° pitch=%+.2f°；足端平面 roll=%+.2f° '
              'pitch=%+.2f°；误差 roll=%+.2f° pitch=%+.2f°；四脚扭曲 %+.2f cm'
              % (path, count, ir, ip, fr, fp, er, ep, tw))
    if not errors:
        sys.exit('没有可用数据')
    er = sum(e[0] for e in errors) / len(errors)
    ep = sum(e[1] for e in errors) / len(errors)
    if len(errors) >= 2:
        print('各文件误差离散（≈地面坡度投影）：roll ±%.2f° pitch ±%.2f°'
              % (max(abs(e[0] - er) for e in errors), max(abs(e[1] - ep) for e in errors)))
    else:
        print('只有一个文件：假设地面水平；建议原地转 180° 再采一次以消去地面坡度。')
    print('采集时安装角 roll=%+.3f° pitch=%+.3f°' % tuple(mount0))
    print('建议安装角   roll=%+.3f° pitch=%+.3f°（yaw 不变）' % (mount0[0] + er, mount0[1] + ep))


if __name__ == '__main__':
    main()
