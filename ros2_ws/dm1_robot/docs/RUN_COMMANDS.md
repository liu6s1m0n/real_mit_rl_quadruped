# DM1 硬件与仿真运行指令

以下命令默认工程路径为：

```text
/home/simon/real_mitrl_dog
```

> 2026-11-03 起：ROS 包名 `mymit_robot` → `dm1_robot`，包目录 `ros2_ws/mit_robot` →
> `ros2_ws/dm1_robot`，仿真程序 `mymit_robot_user` → `sim_main`，配置目录
> `config/algorithm` → `config/opendog`，标定文件移到 `config/hardware/`。
> 旧的 `install/mymit_robot` 已失效，请重新构建并 `source install/setup.bash`。

## 1. 准备 ROS 2 环境

每次打开新的终端，先执行：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
for target in hardware_main sim_main install/fast; do
  CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build \
    --packages-select dm1_robot --symlink-install --parallel-workers 1 --executor sequential \
    --cmake-target "$target" || break
done
source install/setup.bash
```

工程只包含 OpenDog WBC+NMPC 一套控制算法；上述循环逐目标、单线程增量构建
硬件入口和仿真入口，最后安装已构建的程序，不构建全部测试。
`build/`、`install/`、`log/` 均位于 `ros2_ws/` 下。构建需要
`/home/simon/open-dog` 下的 OCS2 安装（`DM1_OPENDOG_PREFIX`），找不到时 CMake 直接报错。
如果只是运行已经构建的程序，可以省略构建循环，但仍需执行两个 `source` 命令。
请在新终端加载 `ros2_ws/install/setup.bash`，不要再加载工程根目录的旧安装环境。

## 2. 仿真程序

### 直接启动 MuJoCo 仿真

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dm1_robot sim_main --render-gpu auto
```

指定其他 OpenDog 配置目录：

```bash
ros2 run dm1_robot sim_main \
  --algorithm-config /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/opendog \
  --render-gpu auto
```

可执行文件先链接 ROS 2 的 `tinyxml2`，再加载 MuJoCo，避免两者全局符号冲突。

使用 launch 时：

```bash
ros2 launch dm1_robot dm1_control.launch.py \
  algorithm_config:=/home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/opendog
```

仿真窗口中按 `Shift+U` 解锁，再按 `1` 进入 OpenDog 站起轨迹；站稳后用
`W/S/A/D/Q/E` 发送运动命令。OpenDog 的 `2` 保持连续原地小跑；`4` 让右前＋左后
各踏一次，随后保持四脚支撑，再次按 `4` 才重新发起一次。`3` 是零点标定动作（见下节）。
窗口按键重复事件不会连续触发；终端收到的每个按键字节均作为新请求。
这些按键在 OpenDog 真机入口使用同样语义。

### 零点标定（3 键）

站起并站稳后按 `3`：四只脚保持不动，机身依次升降 ±4 cm、俯仰 ±10°、横滚 ±8°、
前后 ±4 cm、左右 ±3 cm、偏航 ±12°，共约 32 s，每段开始时终端打印 `1/6 …`，结束时打印
黄色的“零点标定动作完成”，随后回到起始站姿；CSV 中这段的状态标签为 `OPENDOG_ZERO_CAL`。
期间关节参考按四脚不动反解（否则默认站姿的关节权重会抵消平移和升降，
2026-11-13 第一次实机标定前后、左右只动了 0.2/0 cm）。
同时按参考关节速度的方向给四条腿加库仑摩擦补偿（`frictionCompensation.referenceVelocity`，站起爬升段同样使用）：
实机关节静摩擦约 1.2–2 Nm，不补偿时平移/升降只走到命令的 20–40%（仿真加同样摩擦可复现）。
未站立、仍在回零或踏步中按 `3` 不会开始；空格、`1`、`P`
或任何运动键会立即中止。采集后用下面的脚本同时解算 12 个关节零点、IMU 安装角修正和
各脚接地点高度（不要求地面水平），输出建议的标定第 6 列：

```bash
python3 /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/scripts/zero_calibration.py <log.csv> \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt
```

`--calibration` 必须是采集这组数据时用的标定文件。仿真验证（注入 1.4–2.9° 的已知零点误差）：
12 个关节 RMS 0.65°，髋 0.35° 以内；大腿和小腿的误差反向（小腿约 −1.5 × 大腿），
最大 1.2°。四个髋的共同偏差与 IMU 横滚无法区分，只有相对值可信。
用新零点再做一次 `3`，修正量应接近 0，否则说明还有未建模的误差。

单次踏步参数在 `config/opendog/opendog_gait.info` 的 `single_step_lf_rh` 和
`single_step_rf_lh`：先四脚支撑 0.1 s、指定对角腿摆动 0.45 s、四脚支撑 0.1 s，
然后持续站立。沿用原 OpenDog 在预测时域末端插入步态的方式，默认约 1 秒之后
开始动作，另有后台求解延迟。`Space` 发送停止请求。

2026-10-29 本轮仅将两组单次踏步的摆动窗口从 0.30 s 延长到 0.45 s，
连续踏步时序、质量模型和底层 PD 保持原值。当前配置的 MuJoCo 站起和单次踏步
两项测试通过，两组对角腿的测试离地判据分别为 32.6 / 34.3 mm（门槛 15 mm）。
该次测试中的控制模型和仿真机均为 14.705035 kg；连续及重复踏步仍有机身抬高趋势。
用户随后确认当前机器重量已为 14 kg+，后续支撑分析按当前 14.705035 kg 模型核算，
历史 11.8 kg 数据不再作为当前重量基准。

本轮支撑链路复核将 `phaseTransitionStanceTime` 从 0.20 s 调至 0.04 s，
`swingHeight` 从 0.08 m 调至 0.04 m。前者同时影响原有步态支撑过渡和本工程的
模式切换输出混合；后者影响连续与单次踏步。底层 PD 保持 20/5。
实机电机失能保护是否改善仍待新一轮验证。

`[OpenDog][TIMING]` 每秒观测控制计算、WBC、最近一次 MPC 求解耗时、策略观测年龄
和剩余预测时域；真机另有 `[OpenDog][FRESHNESS]`，显示发布时的控制快照年龄、
原始电机反馈最大年龄、策略年龄及剩余时域。这些日志只作观测，不触发保护动作。
策略年龄与仿真/控制时间同轴。与上游 OpenDog `LeggedController::starting()` 一致，进入 OpenDog 控制的第一帧同步求得初始 MPC 策略（约 60 ms，键盘模式只记录一次周期超时），随后由后台 MPC 线程持续更新。
普通仿真和 `--hardware-sim` 都通过同一个 MuJoCo 执行器写入路径。

OpenDog 电机输出增益位于 `config/opendog/opendog_task.info` 的 `hardware_output`。
原源码为 `kp=4`、`kd=5`。当前保留用户实测使用的统一 `kp=20`、`kd=5`，
没有后小腿特殊增益；此前 `kp=12`、`kp=20` 均未解决自主起身问题。
2026-10-29 已将 OpenDog 的质量与惯量恢复到保守估计 **14.705035 kg**：
MuJoCo 模型和 OpenDog URDF 使用同一套 14.705035 kg 链节质量与惯量，
几何、关节零位、目标高度与算法保持不变。OpenDog 使用独立缓存目录
`/tmp/legged_control/dm1_opendog_mjcf_mass14705035`。
用户已确认当前机器重量为 14 kg+；11.8 kg 为此前对照条件。

此前离线验证中，11.8 kg 控制模型驱动 11.8 kg 仿真机的最终高度约 0.277 m；
14.705035 kg 历史模型驱动同一 11.8 kg 仿真机约 0.293 m，存在高度偏差。
当前使用 14.705035 kg 保守模型；模型契约、OpenDog 算法和硬件桥接检查通过，均非实机验证。

2026-10-28 16:34 实机日志 `dm1_march_20261028_163445_199096417417518.csv`
中，未手扶的平稳段（日志时间 8~17 s）高度约 0.2525 m、俯仰 -0.0512 rad，
后膝 RR/RL 的平均目标误差为 0.064/0.103 rad。用户确认后段达到 0.28 m 时有
手扶辅助，不能据此判定已自主站起。该平稳段发布和发送无失败，记录的 WBC 求解成功。
此前试验调整 MPC 的 Q 对角项：高度 `(8,8)` 1500→6000、俯仰 `(10,10)` 300→1200、
横滚 `(11,11)` 300→600（已在下述实机复核后撤回）；统一电机 PD 保持 20/5，
该历史日志使用 14.705035 kg 模型，目标高度、起身时间、摆腿时间和高度、MPC/WBC 实现均未改变。
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
ros2 run dm1_robot sim_main --hardware-sim --render-gpu auto
# ROS launch 等价入口
ros2 launch dm1_robot dm1_control.launch.py hardware_sim:=true
```

该模式调用同一个 `HardwareBridge`、`Dm1HardwareDriver`、`DmMotorDriver`、
`HardwareImu` 和 `OpenDogRobotRunner`。共用启动检查、使能确认、趴卧初始化、
IMU CRC/启动基准、关节标定、OpenDog 状态估计/MPC/WBC、MIT 命令校验/饱和、
双路独立 500 Hz 发送（帧间隔 250 μs）、反馈缓存与故障/旧命令保持策略。
真实 `hardware_main` 的算法修改会自动进入这个模式。

MuJoCo 仅提供机械动力学和设备端替身：CAN 字节交给原始电机驱动解码，
57 字节 DM IMU 包经虚拟串口交给原始 IMU 接收线程。
不会打开物理 CAN 或真实 IMU，不把 MuJoCo 姿态真值交给控制器。
默认读取与实机相同的 `config/hardware/dm1_hardware_calibration.txt`；
可用 `--calibration FILE` 指定同格式文件。

操作顺序与实机键盘模式相同：等待启动反馈通过 → `Enable motors`/`U` →
等待 1.2 秒关节初始化 → `Stand up`/`1` → 站稳后 `2` 或方向按钮；
`Stop`/空格停止，`0` 失能，Esc 退出。暂停或 Reset 会停止旧硬件桥并清空设备命令；
继续后重新执行启动检查，电机保持锁定。站高使用 OpenDog 配置中的 `comHeight`。

默认总力矩上限与当前 `hardware_main` 一样为 **30 Nm**（2026-11-09 由 88 Nm 下调）；
需要其他上限时，仿真和实机要同步指定，例如：

```bash
ros2 run dm1_robot sim_main --hardware-sim --torque-limit 30
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
ros2 run dm1_robot sim_main --hardware-sim --torque-limit 30 \
  --sim-weak-joint 5 --sim-output-scale 0 --sim-impairment-after-s 10
```

模拟每 17 次 MIT 写入失败一次，保留实机其余电机发送/下轮重试/旧命令保持行为：

```bash
ros2 run dm1_robot sim_main --hardware-sim \
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
    --packages-select dm1_robot --symlink-install --parallel-workers 1 --executor sequential \
    --cmake-target "$target" || break
done
source install/setup.bash
ctest --test-dir build/dm1_robot \
  -R '^(test_dm1_full_hardware_simulation|test_hardware_bridge|test_dm_motor_driver|test_imu_driver)$' \
  --output-on-failure
```

该测试覆盖真实串口标定/协议，以及 OpenDog 经原始驱动完成趴卧初始化和站起。

### 使用 ROS 2 launch 启动控制仿真

```bash
ros2 launch dm1_robot dm1_control.launch.py
```

### 只查看模型，不启动控制器

```bash
ros2 launch dm1_robot dm1_display.launch.py
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

### IMU 型号选择

`--imu-type` 选择 IMU 型号，所有硬件模式（`--imu-only`、只读完整模式、键盘控制、
`--set-zero`）都适用：

| 取值 | 型号 | 协议 | `--imu auto` 的查找方式 |
|------|------|------|-------------------------|
| `hfi`（默认） | HandsFree HFI-A9 | `AA 55` + 0x2C 惯性帧 / 0x14 欧拉角帧，CRC16/MODBUS，921600 | `/dev/serial/by-id/*HandsFree*`，其次 `/dev/HFRobotIMU` |
| `dm` | 原 DM IMU | 57 字节三记录帧，921600 | `/dev/serial/by-id/*DM-IMU*` |

两种型号共用同一套启动静止标定、安装偏差补偿（`DM1_IMU_MOUNT_*_DEG`）和加速度
归一化。DM 默认安装偏差为 roll −1.32°、pitch 1.84°；HFI 为 roll 0.97°、pitch −0.07°
（2026-11-06 标定）。

重新标定 HFI 安装偏差：同一位置按 `1` 静止站立 10 s 以上，原地转 180° 再站一次，然后

```bash
python3 dm1_robot/scripts/imu_mount_calibration.py --current-mount 0.97,-0.07 \
  "$DM1_LOG_DIR"/dm1_march_*.csv
```

`--current-mount` 填采集时生效的安装角，脚本输出新的建议值。
MuJoCo 硬件仿真注入的仍是 DM 帧，不受此选项影响。

HFI 的 `/dev/HFRobotIMU` 链接需要安装厂商 udev 规则（只需一次）：

```bash
sudo cp /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/udev/99-handsfree-imu.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

### 自动选择 IMU 串口

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dm1_robot hardware_main \
  --imu-only                    # HFI-A9（默认）
ros2 run dm1_robot hardware_main \
  --imu-only --imu-type dm      # 原 DM IMU
```

`--imu-only` 不需要标定文件，不打开 CAN，也不会发送任何电机命令。

### 指定 IMU 串口

```bash
ros2 run dm1_robot hardware_main \
  --imu-only \
  --imu /dev/serial/by-id/你的-IMU-设备
```

### 使用 launch 启动 IMU

自动选择设备：

```bash
ros2 launch dm1_robot dm1_imu.launch.py
```

指定设备：

```bash
ros2 launch dm1_robot dm1_imu.launch.py \
  port:=/dev/serial/by-id/你的-IMU-设备
```

使用原 DM IMU：`ros2 launch dm1_robot dm1_imu.launch.py imu_type:=dm`。

启动时应保持 IMU 和机器人静止，等待出现 `startup baseline accepted`。
标定时还会打印“姿态/重力一致性”：欧拉角推算的重力方向与加速度计的夹角，超过 3°
会告警。首次使用 HFI 时，把机身向左侧和机头方向各倾斜约 10° 后分别启动一次 IMU-only，
确认该夹角都很小，就说明欧拉角符号约定与加速度计一致。

## 4. 单独监听 DM 电机反馈

### 监听 can0 上物理 CAN ID 为 0x01 的电机

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dm1_robot hardware_main \
  --motor-only \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt \
  --poll-feedback can0 0x01
```

### 监听 can1 上的电机

```bash
ros2 run dm1_robot hardware_main \
  --motor-only \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt \
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
/home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt
```

### 只读启动，默认推荐

```bash
ros2 run dm1_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt
```

完整硬件模式会自动打开 IMU、`can0` 和 `can1`，并等待 12 个电机反馈。只读模式
不会使能电机，也不会写零；通过启动检查后持续监测 IMU。

### 键盘控制完整硬件模式

```bash
ros2 run dm1_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt \
  --keyboard-control
```

该模式启动后始终上锁，且绝不会自动写电机零位。只有在悬空安全支架、急停可用、
机器人静止、水平且 12 个电机反馈完整时，才按 `Shift+U` 解锁。键盘模式不检查当前
关节是否处于零位窗口，操作者必须自行确认姿态安全。解锁失败会失能并退出；`0` 立即
失能，`Esc` 失能后退出。键盘控制与 `--set-zero`、`--imu-only`、`--motor-only` 互斥。

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
stdbuf -oL -eL ros2 run dm1_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt \
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
ros2 run dm1_robot hardware_main --imu-only \
  --imu /dev/serial/by-id/你的-IMU-设备 2>&1 | tee "$DM1_LOG_DIR/imu_only.log"
```

实机推荐顺序：先用 `--motor-only --poll-feedback` 确认反馈，再用只读完整模式确认 IMU
和姿态，最后在物理急停旁执行键盘模式。软件上锁不能替代物理急停。

### 显式执行零位维护

```bash
ros2 run dm1_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt \
  --set-zero
```

`--set-zero` 直接写入全部电机零位，不会使能控制输出，完成后自动退出。
写零前必须把机器人悬空并摆成 OpenDog 机械零位，不能把任意当前姿态当作零位写入：

- 髋（hip）：水平；
- 大腿（thigh）：电机接口与大腿上的螺丝对齐，大腿水平朝后；
- 小腿（calf）：向后伸直并顶到机械限位。

该姿态对应模型角全 0。写零后把机器人放下趴好，再用键盘模式启动；解锁后 1.2 s 内平滑
回到趴卧姿态 `(0, -0.203, -2.22)` rad。

## 6. 模式对照

| 模式 | IMU | CAN | 使能电机 | 写零 | 需要 12 个电机 |
|---|---:|---:|---:|---:|---:|
| `--imu-only` | 是 | 否 | 否 | 否 | 否 |
| `--motor-only --poll-feedback ...` | 否 | 是 | 否 | 否 | 否，可监听单个电机 |
| 完整模式 | 是 | 是 | 否 | 否 | 是 |
| 完整模式 `--set-zero` | 是 | 是 | 否 | 是 | 是 |
| 完整模式 `--keyboard-control` | 是 | 是 | 按键控制 | 否 | 是 |
| MuJoCo 仿真 | 否 | 否 | 按键/GUI 控制 | 否 | 否 |

退出任何硬件程序都应使用 `Ctrl+C`。硬件测试前确认没有另一个程序占用 IMU 串口或
CAN 接口。

### 步态与发送保护

按 `2` 选择 TROT（0.5 秒周期、50% 支撑）。`W/S/A/D/Q/E` 使用方向行走步态；空格停止后仍保留原地踏步选择。仿真与真机使用同一命令仲裁逻辑。

真机由独立线程每 8 ms 发送一批经过校验和力矩限幅的指令。单帧写入失败不会跳过其余电机，下一轮继续尝试；通信/控制失败时持续保持最近一组有效命令，不因超时或通信丢失故障码0x0D自动失能。电机故障保护与操作者显式失能/退出仍保留；电机固件自身保护独立于此软件策略。正常停机先停止发送线程再关闭硬件。该实现不保证非实时Linux的硬截止时间。
