# DM1 硬件与仿真运行指令

以下命令默认工程路径为：

```text
/home/simon/real_mitrl_dog/ros2_ws
```

## 1. 准备 ROS 2 环境

每次打开新的终端，先执行：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot --symlink-install
source install/setup.bash
```

如果只是运行已经成功构建过的程序，可以省略 `colcon build`，但仍需要执行两个
`source` 命令。

## 2. 仿真程序

### 直接启动 MuJoCo 仿真

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mymit_robot mymit_robot_user --render-gpu auto
```

默认启动 MPC/WBC 控制链。仿真窗口打开后默认电机上锁；先按键盘 `Shift+U`（或点击
`Enable motors`），再按 `1`/点击 `Stand up`，最后按方向键或使用方向按钮。方向命令会
锁存，按 `Space` 或点击 `Stop` 显式停止。

### 启动 RL 仿真

```bash
ros2 run mymit_robot mymit_robot_user \
  --walk-mode rl \
  --render-gpu auto
```

### 使用 ROS 2 launch 启动控制仿真

```bash
ros2 launch mymit_robot dm1_control.launch.py
```

使用 RL 模式：

```bash
ros2 launch mymit_robot dm1_control.launch.py walk_mode:=rl
```

### 只查看模型，不启动控制器

```bash
ros2 launch mymit_robot dm1_display.launch.py
```

### 仿真键盘控制

程序从可交互终端启动时启用终端输入；非 TTY 启动时仿真继续运行，但终端输入不可用，
MuJoCo GUI 仍可独立控制。方向键和 GUI 方向按钮都是锁存选择，不依赖系统按键重复；
按 `Space` 或点击 `Stop` 回到 `BalanceStand`，按 `Esc` 上锁并退出仿真。

| 按键 | 功能 |
|---|---|
| `Shift+U` | 电机解锁/使能 |
| `0` | 电机上锁/失能，最高优先级 |
| `1` | 请求站起；行走中回到 BalanceStand |
| `2` | 请求趴卧；由 MPC/WBC 下降并用 Joint-PD 收腿保持，RL 不参与 |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左移 / 右移 |
| `Q` / `E` | 逆时针 / 顺时针原地旋转 |
| `Space` | 停止运动并回到 BalanceStand，不失能 |
| `H` | 打印帮助 |
| `Esc` | 上锁并退出 |

在 RL 仿真中也使用同一个 `2` 趴卧命令；它会从 RL 行走状态直接切换到独立的
`LIE_DOWN` 状态，下降阶段复用现有 MPC/WBC，完成后进入安全的 `JointPd` 保持。

MuJoCo Reset 后会重新上锁；上锁时仍执行 `mj_step()`，机器人会在重力和接触动力学
作用下自然下落，不会暂停物理仿真。

## 3. 单独启动 IMU

### 自动选择 DM IMU 串口

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mymit_robot hardware_main \
  --imu-only
```

`--imu-only` 不需要标定文件，不打开 CAN，也不会发送任何电机命令。

### 指定 IMU 串口

```bash
ros2 run mymit_robot hardware_main \
  --imu-only \
  --imu /dev/serial/by-id/你的-IMU-设备
```

### 使用 launch 启动 IMU

自动选择设备：

```bash
ros2 launch mymit_robot dm1_imu.launch.py
```

指定设备：

```bash
ros2 launch mymit_robot dm1_imu.launch.py \
  port:=/dev/serial/by-id/你的-IMU-设备
```

启动时应保持 IMU 和机器人静止，等待出现 `startup baseline accepted`。

## 4. 单独监听 DM 电机反馈

### 监听 can0 上物理 CAN ID 为 0x01 的电机

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mymit_robot hardware_main \
  --motor-only \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --poll-feedback can0 0x01
```

### 监听 can1 上的电机

```bash
ros2 run mymit_robot hardware_main \
  --motor-only \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --poll-feedback can1 0x01
```

这个模式只发送指定电机的零增益 MIT 轮询帧，用来触发反馈；不会启动 IMU、不会使能
电机、不会写零，也不会向其他电机持续发送 MIT 控制帧。

日志中：

- `UPDATING`：反馈序号持续增长，电机在回复；
- `NO_UPDATE`：暂时没有新的反馈序号；
- `NO_FEEDBACK`：还没有收到有效反馈。

如果只想直接监听 CAN 总线原始帧，也可以使用：

```bash
candump can0
```

DM 电机通常控制帧使用物理 CAN ID，例如 `0x001`，反馈使用 Master ID，例如
`0x011`。不要把 `candump` 当作使能命令，它只读取总线。

## 5. 单个电机使能测试

单电机测试使用独立路径，不会启动 IMU、控制器或其他电机。命令格式为：

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --single-motor JOINT_INDEX BUS CAN_ID \
  --enable-output
```

例如，标定表中第 0 个关节 `FR_hip` 是 `can0` 的物理 ID `4`：

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --single-motor 0 can0 4 \
  --enable-output
```

程序只会向该电机发送一次 `0xFC` 使能命令；使能后不再发送任何 MIT 位置、速度、增益或力矩帧。
按 `Ctrl+C` 会发送 `0xFD` 禁用目标电机后退出。
`JOINT_INDEX`、`BUS` 和 `CAN_ID` 必须与标定文件对应行一致，否则程序拒绝启动。

如需进行明确的低增益动作测试，必须额外添加 `--slow-move-test`：

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --single-motor 3 can0 1 \
  --enable-output \
  --slow-move-test
```

该测试先切换电机到 MIT 模式并重复发送 5 次使能命令，再读取当前位置，然后在 5 秒内移动
`30 deg`（`0.5236 rad`），使用 `Kp=20.0`、`Kd=0.2`、前馈力矩为零，到达目标后立即禁用。程序必须
收到目标电机明确的 `Enabled` 状态才会开始动作；未指定
`--slow-move-test` 时，单电机模式仍然只使能、不发送 MIT 动作帧。

## 6. 完整硬件主程序

标定文件：

```text
/home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt
```

### 只读启动，默认推荐

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt
```

完整硬件模式会自动打开 IMU、`can0` 和 `can1`，并等待 12 个电机反馈。没有
`--enable-output` 时不会使能电机，也不会写零。

### 使能完整控制输出

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --enable-output
```

启动零位策略：

- 偏差 `<0.05 rad`（或 `--zero-tolerance` 指定的窗口）：允许继续，不写零；
- 偏差 `>=` 接受窗口：报初始位置错误，拒绝使能；
- 任何写零操作都必须显式使用 `--set-zero`。

只有完整连接 12 个电机、机器人固定在安全支架上、急停可用时，才允许使用
`--enable-output`。单电机测试不要使用这个命令。

### 键盘控制完整硬件模式

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --keyboard-control
```

该模式启动后始终上锁，且绝不会自动写电机零位。只有在悬空安全支架、急停可用、
机器人静止、水平且 12 个电机反馈完整时，才按 `Shift+U` 解锁。键盘模式不检查当前
关节是否处于零位窗口，操作者必须自行确认姿态安全。解锁失败会失能并退出；`0` 立即
失能，`Esc` 失能后退出。键盘控制与
`--enable-output`、`--stand-up`、`--set-zero`、`--imu-only`、`--motor-only` 互斥。

实机推荐顺序：先用 `--motor-only --poll-feedback` 确认反馈，再用只读完整模式确认 IMU
和姿态，最后在物理急停旁执行键盘模式。软件上锁不能替代物理急停。

可选：使能后请求站起：

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --enable-output \
  --stand-up
```

### 显式执行零位维护

```bash
ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --set-zero
```

`--set-zero` 会在静止、水平且收到 12 路健康反馈后写入并验证全部电机零位，
但不会使能控制输出，完成后自动退出。机器人必须已经摆在确认过的参考姿态，
不能把任意当前姿态当作零位写入。

## 6. 模式对照

| 模式 | IMU | CAN | 使能电机 | 写零 | 需要 12 个电机 |
|---|---:|---:|---:|---:|---:|
| `--imu-only` | 是 | 否 | 否 | 否 | 否 |
| `--motor-only --poll-feedback ...` | 否 | 是 | 否 | 否 | 否，可监听单个电机 |
| 完整模式 | 是 | 是 | 否 | 否 | 是 |
| 完整模式 `--enable-output` | 是 | 是 | 是 | 否 | 是 |
| 完整模式 `--set-zero` | 是 | 是 | 否 | 是 | 是 |
| 完整模式 `--keyboard-control` | 是 | 是 | 按键控制 | 否 | 是 |
| MuJoCo 仿真 | 否 | 否 | 按键/GUI 控制 | 否 | 否 |

退出任何硬件程序都应使用 `Ctrl+C`。硬件测试前确认没有另一个程序占用 IMU 串口或
CAN 接口。
