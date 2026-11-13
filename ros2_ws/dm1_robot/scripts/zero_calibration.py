#!/usr/bin/env python3
"""从 3 键零点标定动作的 CSV 解算 12 个关节零点修正。

标定动作中四只脚踩地不动，机身依次升降、俯仰、横滚、偏航。若某个关节零点有偏差，
按编码器算出的足端会随姿态“漂移”。这里用最小二乘同时求：
  * 12 个关节零点修正 delta（真实角 = 读数 + delta），带高斯先验（缺省 σ = 3°）；
  * 可选的 IMU 安装 roll/pitch 修正；
  * 每只脚的接地点（不假设四只脚等高，因此结果不依赖地面是否平整）；
  * 每个采样时刻的机身平移。
足端为半径 2 cm 的球，模型里按纯滚动处理。

用法：
  python3 scripts/zero_calibration.py <log.csv> --calibration config/hardware/dm1_hardware_calibration.txt
"""
import argparse
import csv
import sys

import numpy as np

LEGS = ('FR', 'FL', 'RR', 'RL')
JOINTS = ('hip', 'thigh', 'calf')
FOOT_RADIUS = 0.02
D2R = np.pi / 180.0


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def foot_body(leg, q):
    """机身系足端球心位置，几何与 dm1_opendog.urdf 一致（同 hardware_bridge.cpp 的 footPositionBody）。"""
    left = leg in ('FL', 'RL')
    rear = leg in ('RR', 'RL')
    side = 1.0 if left else -1.0
    longitudinal = -1.0 if rear else 1.0
    hip = np.array([longitudinal * 0.16175, side * 0.059997, -0.061498])
    hip_to_thigh = np.array([longitudinal * 0.049998, side * 0.019503, 0.0])
    thigh_to_calf = np.array([-0.2, side * 0.07825, 0.0])
    calf_to_foot = np.array([-0.17362, 0.0, -0.14312])
    r0 = rot_x(q[0])
    r1 = r0 @ rot_y(q[1])
    r2 = r1 @ rot_y(q[2])
    return hip + r0 @ hip_to_thigh + r1 @ thigh_to_calf + r2 @ calf_to_foot


def foot_world(rotation, q, leg):
    """世界系（不含机身平移）足端球心减去纯滚动位移，脚不滑时应为常量。"""
    p = rotation @ foot_body(leg, q)
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    pitch = np.arcsin(-np.clip(rotation[2, 0], -1.0, 1.0))
    return p - FOOT_RADIUS * np.array([pitch + q[1] + q[2], -(roll + q[0]), 0.0])


def load_calibration(path):
    table = {}
    for line in open(path, encoding='utf-8'):
        line = line.split('#')[0].strip()
        if not line:
            continue
        name, _bus, _can, _master, direction, zero = line.split()
        table[name] = (int(direction), float(zero))
    return table


def load_samples(path, label, stride):
    reader = csv.reader(open(path))
    header = next(reader)
    index = {name: i for i, name in enumerate(header)}
    cols = ['imu_qw', 'imu_qx', 'imu_qy', 'imu_qz'] + [f'{l}_{j}_q' for l in LEGS for j in JOINTS]
    rows = [[float(r[index[c]]) for c in cols] for r in reader if r[1] == label]
    rows = np.array(rows[::stride])
    if len(rows) == 0:
        return [], np.zeros((0, 4, 3))
    rotations = [quat_to_rot(*(q / np.linalg.norm(q))) for q in rows[:, :4]]
    return rotations, rows[:, 4:].reshape(-1, 4, 3)


def double_center(values):
    """values: (N, 4, ...)。去掉每个采样的机身平移（行均值）和每只脚的接地点（列均值），即二维加性模型的残差。"""
    return (values - values.mean(axis=1, keepdims=True) - values.mean(axis=0, keepdims=True)
            + values.mean(axis=(0, 1), keepdims=True))


def evaluate(rotations, joints, delta, mount, fit_imu):
    """返回足端常量项 f (N,4,3) 以及对零点 (N,4,3,3) 和安装角 (N,4,3,2) 的雅可比。"""
    n = len(rotations)
    f = np.zeros((n, 4, 3))
    jac_q = np.zeros((n, 4, 3, 3))
    jac_m = np.zeros((n, 4, 3, 2))
    for s in range(n):
        rotation = rotations[s] @ rot_x(mount[0]) @ rot_y(mount[1])
        for k, leg in enumerate(LEGS):
            q = joints[s, k] + delta[k]
            f[s, k] = foot_world(rotation, q, leg)
            for j in range(3):
                d = np.zeros(3)
                d[j] = 1e-6
                jac_q[s, k, :, j] = (foot_world(rotation, q + d, leg) - foot_world(rotation, q - d, leg)) / 2e-6
            if fit_imu:
                for j in range(2):
                    dm = mount.copy()
                    dm[j] += 1e-6
                    r_plus = rotations[s] @ rot_x(dm[0]) @ rot_y(dm[1])
                    jac_m[s, k, :, j] = (foot_world(r_plus, q, leg) - f[s, k]) / 1e-6
    return f, jac_q, jac_m


def solve(rotations, joints, prior, meas, fit_imu, iterations=4):
    n_mount = 2 if fit_imu else 0
    delta = np.zeros((4, 3))
    mount = np.zeros(2)
    for _ in range(iterations):
        f, jac_q, jac_m = evaluate(rotations, joints, delta, mount, fit_imu)
        r = double_center(f).reshape(-1)
        cols = []
        for k in range(4):
            for j in range(3):
                full = np.zeros_like(f)
                full[:, k, :] = jac_q[:, k, :, j]
                cols.append(double_center(full).reshape(-1))
        for j in range(n_mount):
            cols.append(double_center(jac_m[..., j]).reshape(-1))
        jac = np.stack(cols, axis=1) / meas
        prior_sigma = np.concatenate([np.full(12, prior), np.full(n_mount, 2.0 * D2R)])
        x = np.concatenate([delta.ravel(), mount[:n_mount]])
        normal = jac.T @ jac + np.diag(1.0 / prior_sigma ** 2)
        gradient = jac.T @ (r / meas) + x / prior_sigma ** 2
        step = -np.linalg.solve(normal, gradient)
        delta += step[:12].reshape(4, 3)
        if fit_imu:
            mount += step[12:14]
    f, _, _ = evaluate(rotations, joints, delta, mount, False)
    residual = double_center(f).reshape(-1)
    sigma = np.sqrt(np.diag(np.linalg.inv(normal)))
    contacts = f.mean(axis=0) - f.mean(axis=(0, 1))   # 各脚接地点（相对平均）
    return delta, mount, sigma, contacts, residual


def baseline_residual(rotations, joints):
    """不修正零点、只去掉机身平移和接地点时的残差，用于对比。"""
    f, _, _ = evaluate(rotations, joints, np.zeros((4, 3)), np.zeros(2), False)
    return double_center(f).reshape(-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csv')
    parser.add_argument('--calibration', required=True, help='采集这组数据时使用的标定文件')
    parser.add_argument('--label', default='OPENDOG_ZERO_CAL', help='CSV 中标定动作的状态标签')
    parser.add_argument('--stride', type=int, default=10, help='每隔多少帧取一个样本（500 Hz）')
    parser.add_argument('--prior-deg', type=float, default=3.0, help='零点修正的先验标准差，度')
    parser.add_argument('--meas-mm', type=float, default=1.0, help='足端位置测量误差，mm')
    parser.add_argument('--no-imu', action='store_true', help='不估计 IMU 安装角修正')
    args = parser.parse_args()

    rotations, joints = load_samples(args.csv, args.label, args.stride)
    if len(rotations) < 100:
        sys.exit(f'{args.label} 样本只有 {len(rotations)} 个，确认按 3 键完整做完了标定动作。')
    meas = args.meas_mm / 1000.0
    base = baseline_residual(rotations, joints)
    delta, mount, sigma, contacts, residual = solve(
        rotations, joints, args.prior_deg * D2R, meas, not args.no_imu)

    print(f'样本 {len(rotations)} 个；足端不动残差 RMS：不修正 {1000 * np.sqrt((base ** 2).mean()):.2f} mm，'
          f'修正后 {1000 * np.sqrt((residual ** 2).mean()):.2f} mm')
    print('\n关节零点修正（真实角 = 读数 + 修正），单位度：')
    calibration = load_calibration(args.calibration)
    weak = []
    for k, leg in enumerate(LEGS):
        cells = []
        for j, joint in enumerate(JOINTS):
            value = np.degrees(delta[k, j])
            spread = np.degrees(sigma[3 * k + j])
            cells.append(f'{joint} {value:+6.2f} ± {spread:.2f}')
            if spread > 0.5 * args.prior_deg:
                weak.append(f'{leg}_{joint}')
        print(f'  {leg}: ' + '   '.join(cells))
    if not args.no_imu:
        print(f'\nIMU 安装角修正：roll {np.degrees(mount[0]):+.2f} ± {np.degrees(sigma[12]):.2f}°，'
              f'pitch {np.degrees(mount[1]):+.2f} ± {np.degrees(sigma[13]):.2f}°（加到 imu.cpp 的安装角上）')
    heights = 100 * (contacts[:, 2] - contacts[:, 2].mean())
    print('\n接地点高度（相对平均，cm）：' + '  '.join(f'{leg} {heights[k]:+.2f}' for k, leg in enumerate(LEGS)) +
          f'   地面扭曲 (FL+RR)-(FR+RL) = {heights[1] + heights[2] - heights[0] - heights[3]:+.2f} cm')
    print('\n建议的标定第 6 列（新值 = 旧值 − 修正 × DIRECTION）：')
    for k, leg in enumerate(LEGS):
        for j, joint in enumerate(JOINTS):
            name = f'{leg}_{joint}'
            direction, zero = calibration[name]
            print(f'  {name:9s} {zero:+.4f} -> {zero - delta[k, j] * direction:+.4f}')
    if weak:
        print('\n注意：以下关节的不确定度超过先验的一半，数据对它们约束不足，修正值主要来自先验：' + ', '.join(weak))


if __name__ == '__main__':
    main()
