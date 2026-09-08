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

默认启动 MPC/WBC 控制链。仿真窗口打开后，先点击 `Stand up`，再点击方向按钮。

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

## 5. 完整硬件主程序

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

- 偏差 `<0.02 rad`：不写零，直接使能；
- 偏差 `[0.02, 0.05) rad`：写入全部 12 个零位，复核成功后使能；
- 偏差 `>=0.05 rad`：报初始位置错误，拒绝使能。

只有完整连接 12 个电机、机器人固定在安全支架上、急停可用时，才允许使用
`--enable-output`。单电机测试不要使用这个命令。

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
| 完整模式 `--enable-output` | 是 | 是 | 是 | 视零位偏差而定 | 是 |
| 完整模式 `--set-zero` | 是 | 是 | 否 | 是 | 是 |
| MuJoCo 仿真 | 否 | 否 | 否 | 否 | 否 |

退出任何硬件程序都应使用 `Ctrl+C`。硬件测试前确认没有另一个程序占用 IMU 串口或
CAN 接口。
