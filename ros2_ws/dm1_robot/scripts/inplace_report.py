#!/usr/bin/env python3
"""
原地踏步实机评估：逐段统计 xyz / rpy 摆动、漂移、水平力需求和支撑脚打滑.

用法：inplace_report.py CSV [CSV ...]
CSV 来自 hardware_main --keyboard-control（dm1_march_*.csv）。每个 OPENDOG_LOCOMOTION 段
丢掉开头 1.5 s 起步过渡后统计；电机反馈中断（feedback_age > 0.1 s）之后的数据自动截掉。

指标含义：
  xyz/rpy σ、峰峰：估计器位姿的标准差和峰峰值（cm / °）。
  漂移：段内 x/y 的线性漂移速度（cm/s）和偏航净变化（°）。
  |Fxy|/Fz：MPC 规划的支撑脚水平力与竖直力之比，中位数和超过摩擦系数 0.25 的比例；
            越大越容易打滑。
  打滑：每个支撑相内足端世界系水平位移的中位数（cm）。
"""

import csv
import math
import statistics
import sys

LEGS = ('FR', 'FL', 'RR', 'RL')
SETTLE_S = 1.5
FRICTION = 0.25


def load(path):
    """读取 CSV，跳过损坏行，并在电机反馈中断处截止."""
    rows = []
    with open(path, newline='') as handle:
        for row in csv.DictReader(handle):
            try:
                float(row['t_s'])
                age = float(row['FR_hip_feedback_age_s'])
            except (TypeError, ValueError):
                continue
            if row['fsm'] == 'OPENDOG_LOCOMOTION' and age > 0.1:
                print('  注意：t=%.2f s 起电机反馈中断，之后的数据不计入' % float(row['t_s']))
                break
            rows.append(row)
    return rows


def segments(rows):
    """按 OPENDOG_LOCOMOTION 连续段切分，返回去掉起步过渡后的行列表."""
    result, current = [], []
    for row in rows:
        if row['fsm'] == 'OPENDOG_LOCOMOTION':
            current.append(row)
        elif current:
            result.append(current)
            current = []
    if current:
        result.append(current)
    trimmed = []
    for seg in result:
        t0 = float(seg[0]['t_s']) + SETTLE_S
        seg = [r for r in seg if float(r['t_s']) >= t0]
        if len(seg) > 500:
            trimmed.append(seg)
    return trimmed


def report(seg):
    """打印一段的统计."""
    g = lambda key: [float(r[key]) for r in seg]
    t = g('t_s')
    duration = t[-1] - t[0]
    parts = []
    for name, key, scale in (('x', 'est_pos_x', 100), ('y', 'est_pos_y', 100), ('z', 'est_pos_z', 100)):
        v = [scale * a for a in g(key)]
        parts.append('%s σ%.1f 峰峰%.1f' % (name, statistics.pstdev(v), max(v) - min(v)))
    print('  位置(cm): ' + ' | '.join(parts) + ' | z 均值 %.3f m' % statistics.mean(g('est_pos_z')))
    parts = []
    for name, key in (('roll', 'body_roll'), ('pitch', 'body_pitch'), ('yaw', 'body_yaw')):
        v = [math.degrees(a) for a in g(key)]
        parts.append('%s σ%.2f 峰峰%.1f' % (name, statistics.pstdev(v), max(v) - min(v)))
    print('  姿态(°):  ' + ' | '.join(parts))
    x, y, yaw = g('est_pos_x'), g('est_pos_y'), g('body_yaw')
    print('  漂移: x %+.1f cm/s  y %+.1f cm/s  偏航净变化 %+.1f°（段长 %.1f s）' % (
        100 * (x[-1] - x[0]) / duration, 100 * (y[-1] - y[0]) / duration,
        math.degrees(yaw[-1] - yaw[0]), duration))
    ratio, slips, start = [], [], {}
    for r in seg:
        for leg in LEGS:
            contact = r['opendog_%s_contact' % leg] == '1'
            fx, fy, fz = (float(r['opendog_%s_force_des_%s' % (leg, a)]) for a in 'xyz')
            if contact and fz > 20:
                ratio.append(math.hypot(fx, fy) / fz)
            p = (float(r['%s_foot_world_x' % leg]), float(r['%s_foot_world_y' % leg]))
            if contact:
                start.setdefault(leg, [p, p])[1] = p
            elif leg in start:
                (x0, y0), (x1, y1) = start.pop(leg)
                slips.append(100 * math.hypot(x1 - x0, y1 - y0))
    if ratio:
        ratio.sort()
        print('  水平力 |Fxy|/Fz 中位 %.3f，超过 %.2f 的比例 %.1f%%；支撑相打滑中位 %.2f cm' % (
            ratio[len(ratio) // 2], FRICTION, 100 * sum(v > FRICTION for v in ratio) / len(ratio),
            statistics.median(slips) if slips else float('nan')))


def main():
    """逐文件、逐段输出."""
    for path in sys.argv[1:]:
        print(path)
        for index, seg in enumerate(segments(load(path)), 1):
            print(' 段 %d：t=%.1f–%.1f s' % (index, float(seg[0]['t_s']), float(seg[-1]['t_s'])))
            report(seg)


if __name__ == '__main__':
    main()
