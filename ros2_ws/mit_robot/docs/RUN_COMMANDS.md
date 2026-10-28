# DM1 硬件与仿真运行指令

以下命令默认工程路径为：

```text
/home/simon/real_mitrl_dog
```

## 1. 准备 ROS 2 环境

每次打开新的终端，先执行：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
for target in hardware_main mymit_robot_user mymit_opendog_user mymit_robot_height_service install/fast; do
  CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
    --packages-select mymit_robot --symlink-install --parallel-workers 1 --executor sequential \
    --cmake-target "$target" --cmake-args -DMYMIT_ENABLE_OPENDOG=ON || break
done
source install/setup.bash
```

两套算法属于同一个 `mymit_robot` 包；上述循环逐目标、单线程增量构建硬件入口、
MIT/OpenDog 仿真入口及高度服务，最后安装已构建的程序，不构建全部测试。
`build/`、`install/`、`log/` 均位于 `ros2_ws/` 下，MIT 默认保留，OpenDog 用参数选择。
如果只是运行已经构建的程序，可以省略构建循环，但仍需执行两个 `source` 命令。
请在新终端加载 `ros2_ws/install/setup.bash`，不要再加载工程根目录的旧安装环境。

## 2. 仿真程序

### 直接启动 MuJoCo 仿真

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mymit_robot mymit_robot_user --render-gpu auto
```

选择 OpenDog WBC+NMPC 仿真；当前 MIT 算法仍是默认值：

```bash
ros2 run mymit_robot mymit_robot_user \
  --algorithm opendog \
  --algorithm-config /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/algorithm \
  --render-gpu auto
```

该选择会由原入口转交同目录的 `mymit_opendog_user` 进程，隔离 ROS2 与 MuJoCo
的 `tinyxml2` 符号顺序；`current_mit` 继续留在原 `mymit_robot_user` 进程。

使用 launch 时选择同一个参数：

```bash
ros2 launch mymit_robot dm1_control.launch.py \
  algorithm:=opendog \
  algorithm_config:=/home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/algorithm
```

仿真窗口中按 `Shift+U` 解锁，再按 `1` 进入 OpenDog 站起轨迹；站稳后用
`W/S/A/D/Q/E` 发送运动命令。OpenDog 的 `2` 保持连续原地小跑；`3` 让左前＋右后
各踏一次，`4` 让右前＋左后各踏一次，随后保持四脚支撑。再次按 `3` 或 `4` 才重新
发起一次；窗口按键重复事件不会连续触发。终端收到的每个按键字节均作为新请求。
这两键在 OpenDog 真机入口使用同样语义，MIT 的原有控制逻辑保持不变。

单次踏步参数在 `config/algorithm/opendog_gait.info` 的 `single_step_lf_rh` 和
`single_step_rf_lh`：先四脚支撑 0.1 s、指定对角腿摆动 0.3 s、四脚支撑 0.1 s，
然后持续站立。沿用原 OpenDog 在预测时域末端插入步态的方式，默认约 1 秒之后
开始动作，另有后台求解延迟。`Space` 发送停止请求。

`[OpenDog][TIMING]` 每秒观测控制计算、WBC、最近一次 MPC 求解耗时、策略观测年龄
和剩余预测时域；真机另有 `[OpenDog][FRESHNESS]`，显示发布时的控制快照年龄、
原始电机反馈最大年龄、策略年龄及剩余时域。这些日志只作观测，不触发保护动作。
策略年龄与仿真/控制时间同轴；首次策略在已有 MPC 线程准备，准备期间保持实测关节位置、零期望速度和零前馈，不阻塞硬件控制循环。
`current_mit` 和 `opendog` 都通过同一个 MuJoCo 执行器写入路径。

OpenDog 电机输出增益位于 `config/algorithm/opendog_task.info` 的 `hardware_output`。
原源码为 `kp=4`、`kd=5`。当前保留用户实测使用的统一 `kp=20`、`kd=5`，
没有后小腿特殊增益；此前 `kp=12`、`kp=20` 均未解决自主起身问题。
2026-10-28 按用户要求恢复提交 `2ed1771`（`f3c1696` 的父提交）的质量与惯量：
整机模型为 **14.705035 kg**，实测整机仍为 **11.8 kg**。MIT 参数模型、MuJoCo
模型和 OpenDog URDF 同步恢复，几何、关节零位、目标高度与算法保持不变。
OpenDog 使用独立缓存目录 `/tmp/legged_control/dm1_opendog_mjcf_mass14705035`，
首次运行会生成新质量对应的动力学库。该模型比实测质量高约 24.6%，属于用户要求的
历史参数对比；不能把增加支撑需求直接视为已修复实机起身。

本次离线验证：统一 `kp=20/kd=5` 时，11.8 kg 控制模型驱动 11.8 kg 仿真机的
最终高度约 0.277 m；14.705035 kg 控制模型驱动同一 11.8 kg 仿真机约 0.293 m，
存在高度偏差。控制模型和仿真机均恢复历史质量后，完整原始驱动起身测试通过，
最终约 0.277 m。模型、OpenDog 算法和硬件桥接检查通过。MIT 完整驱动踏步回归的
最低高度约 0.262 m，未达到测试要求的 0.27 m，此项仍失败。均非实机验证。

2026-10-28 16:34 实机日志 `dm1_march_20261028_163445_199096417417518.csv`
中，未手扶的平稳段（日志时间 8~17 s）高度约 0.2525 m、俯仰 -0.0512 rad，
后膝 RR/RL 的平均目标误差为 0.064/0.103 rad。用户确认后段达到 0.28 m 时有
手扶辅助，不能据此判定已自主站起。该平稳段发布和发送无失败，记录的 WBC 求解成功。
此前试验调整 MPC 的 Q 对角项：高度 `(8,8)` 1500→6000、俯仰 `(10,10)` 300→1200、
横滚 `(11,11)` 300→600（已在下述实机复核后撤回）；统一电机 PD 保持 20/5，质量和惯量保持 14.705035 kg
历史模型，目标高度、起身时间、摆腿时间和高度、MPC/WBC 实现均未改变。
同一实测反馈的离线回放中，RR/RL 膝目标角平均增加约 0.036/0.038 rad，指令力矩
峰值约 19.0→22.0 Nm；这是同反馈下的输出比较，不能代替闭环实机结果。
20 N 后部加载仿真和两组对角单次踏步测试通过，但下述实机复测未证实改善。
此日志进入踏步后还有发送失败、反馈变旧；其后的掉落不能单凭站立参数解释。

2026-10-28 16:59 与 17:00 两次站立实机复核：用户确认手扶动作是向上抬后部、
使机身变平；对应俯仰角从负值回到零，与动作一致。IMU 俯仰与重力方向、
角度变化与陀螺仪同号，未发现需要翻转 IMU 方向的证据。最新日志
`dm1_march_20261028_170016_200627038930336.csv` 的 8~12 s 内，
关节目标经模型正运动学换算，前脚相对实测位置上移约 4~8 mm，后脚下移约
22~27 mm，属于前腿收、后腿伸的纠偏方向；这只是目标方向，不代表真实足端已运动。
新权重下未手扶站立俯仰约 -0.080 rad，较旧日志约 -0.051 rad 更差；
同一 8~12 s 窗口右后膝指令力矩波动标准差约 0.25→0.53 Nm，
有约 10 Hz 波动，与约 100 ms 的 MPC 实际求解周期接近，尚不能据此断言全部根因。
两次站立日志均无命令发布或电机发送失败。已恢复 Q 高度/俯仰/横滚为
**1500/300/300**，保留 PD 20/5、历史质量、控制周期与算法实现。
回退后 IMU 驱动、姿态估计、OpenDog 接口三项测试，以及原始驱动解码和
OpenDog 双后膝展开两项仿真测试通过；后者最终高度约 0.277 m，属于仿真结果。
后腿尚未自主完全展开的问题仍未解决，回退权重不能视为实机修复完成。

启动时 `[OpenDog][OUTPUT]` 打印实际加载的两个增益值。
`[OpenDog][WBC]` 每秒记录 QP 求解/取解返回码（0 表示成功）、接触模式，
以及每条腿的 MPC 期望支撑力和 WBC 支撑力；仅用于诊断。
硬件控制和电机发送均保持原工程的 2 ms（500 Hz）；CSV 逐周期快照由后台线程
完成格式化和写盘，退出时排空日志队列。超时日志的 `poll_slot` 只表示轮询时隙，
`command_published` 表示本周期是否将有效命令交给发送线程。

默认启动 MPC/WBC 控制链。仿真窗口打开后默认电机上锁；先按键盘 `Shift+U`（或点击
`Enable motors`），再按 `1`/点击 `Stand up`，最后按方向键或使用方向按钮。方向命令会
锁存，按 `Space` 或点击 `Stop` 显式停止。

### 完整实机算法仿真（实机联调优先使用）

```bash
ros2 run mymit_robot mymit_robot_user --hardware-sim --render-gpu auto
# ROS launch 等价入口
ros2 launch mymit_robot dm1_control.launch.py hardware_sim:=true
```

该模式调用同一个 `HardwareBridge`、`Dm1HardwareDriver`、`DmMotorDriver`、
`HardwareImu` 和硬件版 `RobotRunner`。共用启动检查、使能确认、零位初始化、
IMU CRC/启动基准/融合、关节标定、FSM、MPC/WBC、MIT 命令校验/饱和、
双路独立 500 Hz 发送（帧间隔 250 μs）、反馈缓存与故障/旧命令保持策略。
真实 `hardware_main` 的算法修改会自动进入这个模式。

MuJoCo 仅提供机械动力学和设备端替身：CAN 字节交给原始电机驱动解码，
57 字节 DM IMU 包经虚拟串口交给原始 IMU 接收线程。
不会打开物理 CAN 或真实 IMU，不把 MuJoCo 姿态真值交给控制器。
默认读取与实机相同的 `config/dm1_hardware_calibration.txt`；
可用 `--calibration FILE` 指定同格式文件。

操作顺序与实机键盘模式相同：等待启动反馈通过 → `Enable motors`/`U` →
等待 1.2 秒关节初始化 → `Stand up`/`1` → 站稳后 `2` 或方向按钮；
`Stop`/空格停止，`0` 失能，Esc 退出。暂停或 Reset 会停止旧硬件桥并清空设备命令；
继续后重新执行启动检查，电机保持锁定。高度滑块在该模式不显示，
因为当前完整实机程序使用固定站起目标和 MPC/WBC。

默认总力矩上限与当前 `hardware_main` 一样为 **88 Nm**；
复现今天以 30 Nm 进行的实机测试时必须同步指定：

```bash
ros2 run mymit_robot mymit_robot_user --hardware-sim --torque-limit 30
```

设备模型包含协议量化、命令保持、速度相关力矩上限和一阶电机响应。
以下默认时序是**未实测标定的压力测试假设**，不是 DM1 实测参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--sim-command-delay-ms` | 1 | CAN 写入到电机接收 |
| `--sim-feedback-delay-ms` | 1 | 电机采样到反馈接收 |
| `--sim-imu-period-ms` | 2 | IMU 数据包产生周期 |
| `--sim-imu-delay-ms` | 2 | IMU 采样到串口送达 |
| `--sim-motor-time-constant-ms` | 4 | 电机力矩响应时间常数 |

物理步长为 2 ms，设备送达时间向下一物理 tick 对齐；操作系统调度和渲染也会产生延迟。
模型的质量、摩擦、柔性、供电压降、温升、电机固件内部保护和传感器噪声仍需实测辨识，
本模式不保证真实机器与仿真机械运动完全一致。独立发送器保持旧命令的行为也完整保留，
可从日志的 command_age 和跟踪误差观察后果。

模拟 FL_calf（索引 5）在使能 10 秒后输出丢失，检查“正常发帧但关节不跟踪”：

```bash
ros2 run mymit_robot mymit_robot_user --hardware-sim --torque-limit 30 \
  --sim-weak-joint 5 --sim-output-scale 0 --sim-impairment-after-s 10
```

模拟每 17 次 MIT 写入失败一次，保留实机其余电机发送/下轮重试/旧命令保持行为：

```bash
ros2 run mymit_robot mymit_robot_user --hardware-sim \
  --sim-drop-every 17 --sim-impairment-after-s 2
```

`--sim-output-scale` 取 0～1；异常计时从本轮第一次使能开始。
索引顺序为 FR/FL/RR/RL，每腿 hip/thigh/calf。
故障场景是敏感性验证，不能作为已确认的实机故障原因。
日志沿用实机的 `/tmp/dm1_march_*.csv`，记录估计姿态、关节目标、实际反馈及力矩。

构建并验证完整原始驱动闭环：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
for target in test_dm1_full_hardware_simulation test_hardware_bridge test_dm_motor_driver test_imu_driver; do
  CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
    --packages-select mymit_robot --symlink-install --parallel-workers 1 --executor sequential \
    --cmake-target "$target" --cmake-args -DMYMIT_ENABLE_OPENDOG=ON || break
done
source install/setup.bash
ctest --test-dir build/mymit_robot \
  -R '^(test_dm1_full_hardware_simulation|test_hardware_bridge|test_dm_motor_driver|test_imu_driver)$' \
  --output-on-failure
```

原有 `test_dm1_hardware_path_comparison` 只比较理想传感器下的 RobotRunner；
它不覆盖原始驱动和设备时序。新的完整测试覆盖真实串口标定/协议、
站起/原地小跑/停止、FL 小腿无输出和间歇写入失败。

### 使用 ROS 2 launch 启动控制仿真

```bash
ros2 launch mymit_robot dm1_control.launch.py
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
| `2` | 原地踏步：对角小跑（TROT，0.5 秒周期、50% 支撑，FR+RL 与 FL+RR 交替抬起） |
| `P` | 请求趴卧；由 MPC/WBC 下降并用 Joint-PD 收腿保持 |
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左移 / 右移 |
| `Q` / `E` | 逆时针 / 顺时针原地旋转 |
| `Space` | 停止运动并回到 BalanceStand，不失能 |
| `H` | 打印帮助 |
| `Esc` | 上锁并退出 |

`P` 趴卧命令会切换到独立的 `LIE_DOWN` 状态，下降阶段复用现有 MPC/WBC，完成后进入 `JointPd` 保持。

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

### 实机站立偏移诊断采集

增强诊断 CSV 会记录每个控制周期的 IMU 序号、姿态四元数、加速度、角速度、重力模长，
状态估计的位置/速度/角速度/世界加速度，四条腿足端的机身系和世界系完整坐标，以及
12 个关节的实际值、目标值、反馈力矩、前馈力矩、PD 分量和已发布帧。采集不改变控制命令。

先在工作空间准备环境，并将日志集中到一个目录：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DM1_LOG_DIR="$HOME/dm1_diag_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$DM1_LOG_DIR"
stdbuf -oL -eL ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --keyboard-control 2>&1 | tee "$DM1_LOG_DIR/terminal.log"
```

确认启动检查通过后，保持急停可用，按 `Shift+U` 解锁，等待关节初始化完成，再按 `1` 站立；
让机器保持站立 10～20 秒，不要按方向键。若持续向一侧偏，立即按 `0`，再按 `Ctrl+C`。
CSV 会在终端打印实际路径。随后可打包：

```bash
tar -czf "${DM1_LOG_DIR}.tar.gz" -C "$(dirname "$DM1_LOG_DIR")" "$(basename "$DM1_LOG_DIR")"
```

另外做一次不接触电机的 IMU 轴向测试；机器人保持断电或安全固定，分别轻微抬高左侧和机头，
观察终端里的 `acc` 与 `rpy_deg` 符号，然后按 `Ctrl+C`：

```bash
ros2 run mymit_robot hardware_main --imu-only \
  --imu /dev/serial/by-id/你的-DM-IMU-设备 2>&1 | tee "$DM1_LOG_DIR/imu_only.log"
```

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

### 步态与发送保护

按 `2` 选择 TROT（0.5 秒周期、50% 支撑）。`W/S/A/D/Q/E` 使用方向行走步态；空格停止后仍保留原地踏步选择。仿真与真机使用同一命令仲裁逻辑。

真机由独立线程每 8 ms 发送一批经过校验和力矩限幅的指令。单帧写入失败不会跳过其余电机，下一轮继续尝试；通信/控制失败时持续保持最近一组有效命令，不因超时或通信丢失故障码0x0D自动失能。电机故障保护与操作者显式失能/退出仍保留；电机固件自身保护独立于此软件策略。正常停机先停止发送线程再关闭硬件。该实现不保证非实时Linux的硬截止时间。
