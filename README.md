面向达妙 DM1 四足机器人的 ROS 2 工程，C++ 实现，控制算法为 OpenDog WBC+NMPC：
Kalman 状态估计、OCS2 SQP 非线性模型预测控制（NMPC）和加权全身控制（WBC）。
同一套算法既跑 MuJoCo 仿真，也通过达妙 MIT 电机协议驱动真机。

- 完整运行指令、按键表和安全顺序：[RUN_COMMANDS.md](ros2_ws/dm1_robot/docs/RUN_COMMANDS.md)
- 架构与 OpenDog 源码依据：[hardware_algorithm_separation.md](ros2_ws/dm1_robot/docs/hardware_algorithm_separation.md)
- 电机零位标定：[OPENDOG_ZERO_CALIBRATION.md](ros2_ws/dm1_robot/docs/OPENDOG_ZERO_CALIBRATION.md)

## 控制流程

```text
MuJoCo 仿真 / 真机 IMU + 电机反馈
          ↓
  Kalman 状态估计（OpenDog）
          ↓
  SQP-MPC（OCS2，后台线程求解）
          ↓
      WeightedWbc
          ↓
  关节 q/dq/τ + Kp/Kd → 达妙 MIT 电机协议 / MuJoCo 执行器
```

## 工程结构

```text
MY_ROBOT/
├── ros2_ws/
│   └── dm1_robot/                 # ROS 2 包 dm1_robot
│       ├── config/
│       │   ├── opendog/           # OpenDog 算法配置：URDF、task/reference/gait
│       │   └── hardware/          # 电机标定文件
│       ├── description/           # MuJoCo 模型 dm1.xml、scene.xml 与 STL 网格
│       ├── launch/                # ROS 2 启动文件
│       ├── include/  src/         # 两者按同样的模块分类：
│       │   ├── common/            #   公共类型、参数配置、日志（仅头文件）
│       │   ├── control/           #   OpenDog 算法接入与真机 runner
│       │   ├── hardware/          #   硬件桥、达妙电机 MIT 协议、CAN/IMU 驱动
│       │   ├── sensor/            #   IMU 与关节反馈数据源
│       │   ├── teleop/            #   键盘遥操作与电机输出仲裁
│       │   ├── simulation/        #   MuJoCo 仿真桥与设备替身
│       │   └── app/               #   程序入口 hardware_main.cpp、sim_main.cpp（仅 src）
│       ├── third_party/
│       │   ├── opendog/           # OpenDog WBC、OCP 接口、估计器和 SQP-MPC 源码
│       │   ├── dmbot_serial/      # 达妙 CAN 协议
│       │   ├── dm_imu/            # 达妙 IMU CRC
│       │   └── legacy/            # ROS 1 遗留包，仅作参考，不参与构建
│       ├── test/                  # 按模块划分的单元与集成测试
│       ├── scripts/               # CAN 配置与数据采集脚本
│       └── docs/                  # 运行指令、零位标定、架构说明
└── README.md
```

## 环境依赖

- Linux + ROS 2 Humble，C++17 编译器，CMake 与 colcon
- MuJoCo（`/usr/local` 下的 3.x，含 Simulate UI）、Eigen3、OpenGL 与 GLFW
- OpenDog 的 OCS2 安装：默认 `/home/simon/open-dog/ros2_ws/install` 和
  `/home/simon/open-dog/.ros2_overlay`（CMake 变量 `DM1_OPENDOG_PREFIX` /
  `DM1_OPENDOG_OVERLAY` 可改）；找不到时 CMake 直接报错

## 构建与测试

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dm1_robot --symlink-install
source install/setup.bash

colcon test --packages-select dm1_robot
colcon test-result --verbose
```

修改代码后重新执行 `colcon build` 和 `source install/setup.bash`；怀疑运行的是旧程序时
再执行一次 `hash -r`。

## 运行仿真

```bash
ros2 run dm1_robot sim_main --render-gpu auto
# 或
ros2 launch dm1_robot dm1_control.launch.py
```

机器人从趴卧姿态原地开始。先点击 `Enable motors`（或 `Shift+U`），再点击
`Stand up`（或 `1`），站稳后用方向按钮或 `W/S/A/D/Q/E` 运动。方向命令会锁存，
按 `Space` 或点击 `Stop` 显式停止。

加 `--hardware-sim` 时，MuJoCo 只充当 CAN 电机和 IMU 串口的设备替身，控制部分运行的
是与真机完全相同的 `HardwareBridge` 程序，适合实机前联调。仅查看模型（不带控制器）
用 `ros2 launch dm1_robot dm1_display.launch.py`。

## 真机准备

### IMU 串口

真实 IMU 由 `HardwareImu` 直接读取串口（不启动单独的 ROS IMU 节点），只接收、
不向 IMU 发送配置命令；IMU 需预先配置为持续输出 57 字节数据帧。串口参数为
921600 波特率 8N1。默认从 `/dev/serial/by-id` 自动选择名称含 `DM-IMU` 的设备，
也可用 `--imu DEVICE` 显式指定。

当前用户需要属于 `dialout` 组（只需执行一次，之后注销重新登录，或在当前终端
`newgrp dialout`）：

```bash
sudo usermod -aG dialout "$USER"
```

### CAN（双路 USB2CANFD，gs_usb 固件）

刷入 `gs_usb` 固件后直接提供 `can0`/`can1` SocketCAN 接口，不使用 `slcand`。
设备参数：FDCAN 模式，仲裁域 1 Mbps（采样点 75%），数据域 5 Mbps（采样点 87.5%）。
DM 电机使用 8 字节经典 CAN 帧，实际只用到仲裁域 1 Mbps。

```bash
sudo /home/simon/real_mitrl_dog/ros2_ws/dm1_robot/scripts/setup_dm1_can.sh can0 can1
```

脚本等价于对 `can0`、`can1` 分别执行：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can \
  bitrate 1000000 sample-point 0.75 \
  dbitrate 5000000 dsample-point 0.875 \
  fd on loopback off
sudo ip link set can0 up
```

该设备不支持 `restart-ms`；发送队列异常时用 `ip link set canX down/up` 手动恢复，
用 `ip -details -statistics link show can0` 查看实际参数。

## 真机运行

以下命令都在已 `source install/setup.bash` 的终端中执行。标定文件为
`ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt`，下文简写为 `$CAL`：

```bash
CAL=/home/simon/real_mitrl_dog/ros2_ws/dm1_robot/config/hardware/dm1_hardware_calibration.txt
```

推荐按下面的顺序逐步确认，软件上锁不能替代物理急停。

### 1. 只测 IMU

不需要标定文件，不打开 CAN，不发送任何电机命令：

```bash
ros2 run dm1_robot hardware_main --imu-only
# 等价：ros2 launch dm1_robot dm1_imu.launch.py
```

启动时会先采集静止基准（3 秒内需收到足够稳定的帧，期间保持 IMU 静止），成功时打印
`[DM IMU] startup baseline accepted: ...`，随后周期性输出 `status=UPDATING`、
递增的 `seq` 以及加速度、角速度和欧拉角。

### 2. 只测电机反馈

部分达妙电机只在收到 MIT 帧后才回一帧反馈。下面的命令只向指定电机周期性发送
`kp=0, kd=0, tau=0` 的 MIT 帧，不使能、不写零，也不向其他电机发 MIT 帧；退出时
仍会向标定地址发送失能帧。

```bash
ros2 run dm1_robot hardware_main --motor-only --calibration "$CAL" --poll-feedback can0 0x01
```

终端约每 100 ms 输出一次：`UPDATING` 表示 `seq` 在增长，`NO_UPDATE` 表示暂时没有
新反馈，`NO_FEEDBACK` 表示尚未收到有效反馈帧。电机规划为物理 CAN ID `0x01`～`0x06`、
Master ID `0x11`～`0x16`：控制/使能/失能/写零帧用物理 ID 作仲裁 ID，反馈帧用
Master ID，并在 D0 低 4 位携带物理 ID。

### 3. 完整硬件只读启动

打开 IMU 和两路 CAN、等待 12 路电机反馈，通过启动检查后持续监测 IMU；不使能电机：

```bash
ros2 run dm1_robot hardware_main --calibration "$CAL"
```

### 4. 键盘控制

```bash
ros2 run dm1_robot hardware_main --calibration "$CAL" --keyboard-control
```

启动后电机上锁。启动检查要求 12 路反馈健康、机器人静止且水平，**不检查关节是否处于
趴卧姿态**，按 `Shift+U` 解锁前必须人工确认机器人已趴好。解锁后 1.2 s 内平滑回到
趴卧姿态 `(0, -0.203, -2.22)`，再按 `1` 站立；`P` 趴下，`0` 失能，`Esc` 失能并退出。
完整按键表见 RUN_COMMANDS.md。

### 5. 电机零位维护

电机零位就是 OpenDog 机械零位：髋水平；大腿电机接口对准螺丝（大腿水平朝后）；
小腿向后伸直顶到限位。DM1 脚着地时大腿无法与躯干平行，因此写零时机器人必须悬空，
摆成这个姿态并固定后执行：

```bash
ros2 run dm1_robot hardware_main --calibration "$CAL" --set-zero
```

`--set-zero` 不检查姿态、直接写零，也不使能输出。只读启动和键盘解锁都绝不会写入
电机参数。参考图和步骤见 OPENDOG_ZERO_CALIBRATION.md。

## 常见报错

程序通常先打印具体原因，最后的 `DM1 硬件启动失败: ...` 只是汇总，应优先看前几行。

| 日志 | 原因和处理 |
|---|---|
| `[DM IMU] open(...) failed: Permission denied` | 没有串口权限。用 `id` 确认是否有 `dialout`；没有则 `newgrp dialout` 或注销重新登录。不要用 `chmod 666` 作永久方案。 |
| `[DM IMU] open(...) failed: No such file or directory` | 设备不存在、USB 松动或插拔后设备名变化。用 `ls -l /dev/ttyACM* /dev/ttyUSB*` 确认后修改 `--imu`。 |
| `[DM IMU] open(...) failed: Device or resource busy` | 串口被调试器、厂商工具或旧进程占用。用 `fuser -v /dev/ttyACM0` 找到并关闭占用者。 |
| `[DM IMU] tcgetattr/tcsetattr failed` | 不是可配置的串口、设备已断开，或不支持 921600 波特率。重新插拔并确认设备路径。 |
| `no serial bytes received` | 串口已打开但 IMU 没发数据。检查供电、发送功能、TX/RX/GND 接线和波特率。 |
| `but no valid 57-byte frame` | 收到数据但帧格式或波特率不匹配。确认 IMU 输出 57 字节协议帧、波特率 921600。 |
| `[DM IMU] startup baseline rejected` | 启动静止基准未通过：机器人在动，或 IMU 数据不稳定。保持静止后重试。 |
| `无法打开标定文件` | 标定文件路径错误。使用绝对路径；首次可先用 `--imu-only` 验证 IMU。 |
| `DM1 bus must be 0/1, physical CAN ID in [1,0x0f] ...` | 标定文件中的总线或 ID 越界。物理 CAN ID 为 1～0x0F，Master ID 为 1～0x7FF。 |
| `DM1 motor bus/CAN/master mappings must be unique` | 标定文件中同一总线上有重复的物理 ID 或 Master ID。 |
| `启动检查被拒绝：未收到 IMU 和 12 个电机的完整首轮反馈` | 500 ms 内没有收齐反馈。检查 CAN 配置、电机供电和标定文件中的地址。 |
| `启动检查被拒绝：机器人必须静止、水平，且反馈健康` | 启动时机器人在动、倾斜过大，或有电机报故障/过温。 |

## 新增硬件驱动

新增或替换 CAN/IMU 设备时实现 `dm1_hardware::Dm1HardwareIo`（见
`include/hardware/dm1_hardware_io.hpp`），不要绕过 `Dm1MitInterface` 的命令校验和
力矩饱和。

## License

本项目包含采用 Apache-2.0 与 BSD-3-Clause 许可证发布的代码及资源，具体授权信息请参阅
工程中的 [LICENSE](ros2_ws/dm1_robot/LICENSE) 文件及相关模型目录中的许可说明。
