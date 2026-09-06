# MY_ROBOT

MY_ROBOT 是一个面向四足机器人运动控制与仿真研究的 ROS 2 工程。项目以 C++ 为主要开发语言，结合 MuJoCo 构建机器人仿真环境，并集成状态估计、步态规划、有限状态机、模型预测控制（MPC）和全身控制（WBC）等模块。

仓库当前以 DM1 为唯一机器人模型，用于控制算法开发、MuJoCo 验证、RL
策略回放和真实 DM1 执行器接入。

## 主要功能

- 基于 MuJoCo 的四足机器人动力学仿真与可视化
- DM1 唯一模型契约：MuJoCo、控制核心、RL 和硬件共用关节顺序与限幅
- 腿部控制、足端轨迹规划与步态调度
- 姿态、位置、速度及接触状态估计
- 基于有限状态机的机器人行为管理与安全检查
- MPC 与 WBC 运动控制框架
- ROS 2 自定义消息、服务和启动文件
- 覆盖模型、传感器、估计器、控制器及系统集成的自动化测试

## 控制框架

工程中的主要控制流程如下：

```text
MuJoCo 仿真 / 传感器数据
          ↓
      状态估计
          ↓
      控制状态机
          ↓
      MPC / WBC
          ↓
      腿部控制器
          ↓
     四足机器人模型
```

## 工程结构

```text
MY_ROBOT/
├── ros2_ws/
│   └── mit_robot/
│       ├── include/       # 控制、模型、估计与工具类头文件
│       ├── src/           # 核心算法及机器人模型资源
│       ├── user/          # 仿真程序、运行器与 ROS 2 服务
│       ├── msg/           # ROS 2 自定义消息
│       ├── srv/           # ROS 2 自定义服务
│       └── test/          # 单元测试与集成测试
└── README.md
```

## 环境依赖

建议在 Linux 环境下使用，主要依赖包括：

- ROS 2（当前工程面向 Humble 环境）
- C++17 编译器
- CMake 与 colcon
- MuJoCo
- Eigen3
- OpenGL 与 GLFW

此外，机器人描述与可视化功能会使用 Xacro、RViz2、robot_state_publisher 及 ros2_control 等 ROS 2 组件。

## DM IMU

真实硬件 IMU 由 `ros2_ws/mit_robot/src/sensor/imu.cpp` 中的 `HardwareImu`
直接读取和处理，不再启动独立的 ROS 2 IMU 接收节点，也不发布 `/imu/data`。
程序只从串口接收，不向 IMU 发送配置命令；IMU 需要预先配置为持续输出三段
57 字节数据帧。

### 启动前准备

先连接 IMU，然后确认系统分配的串口设备名：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

工程默认使用 `/dev/ttyACM0`，串口参数为 921600 波特率、8 数据位、无校验、1
停止位（8N1）。如果实际显示的是 `/dev/ttyACM1` 或 `/dev/ttyUSB0`，启动命令中
必须替换为实际设备名。

当前用户需要属于 `dialout` 组。该配置只需执行一次：

```bash
sudo usermod -aG dialout "$USER"
```

执行后需要注销并重新登录；也可以只对当前终端临时生效：

```bash
newgrp dialout
id
```

`id` 输出中应包含 `dialout`。设备每次插拔后不需要再次执行
`usermod`，但要重新确认设备名是否发生变化。

### 仅测试 IMU 接收（推荐首次使用）

该模式不需要标定文件，也不会打开电机 CAN，更不会发送任何电机命令：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot --symlink-install
source install/setup.bash

ros2 run mymit_robot hardware_main \
  --imu-only \
  --imu /dev/ttyACM0
```

看到以下日志，说明串口已经打开并启动接收线程：

```text
[DM IMU] receiver started: /dev/ttyACM0, 921600 baud
```

之后应周期性看到 `frame #... received` 和加速度、角速度、欧拉角数据。按
`Ctrl+C` 退出。

### 完整硬件程序

完整模式还需要有效的电机标定文件。不要直接使用示例路径
`path/to/calibration.txt`，应替换成真实文件的绝对路径：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run mymit_robot hardware_main \
  --calibration /home/simon/real_mitrl_dog/ros2_ws/mit_robot/config/dm1_hardware_calibration.txt \
  --imu /dev/ttyACM0
```

当前 `hardware_main` 只接入了 IMU；CAN/12 路电机反馈仍保持安全拒绝，
因此不会用伪造的腿部数据启动真机控制。不要使用 `--enable-output`，除非已经
确认电机 ID、方向、机械零位和急停措施。

### 常见报错排查

程序通常会先打印 `[DM IMU] ...` 的具体原因，最后的
`DM1 hardware startup failed` 只是汇总错误。应优先查看前一行。

| 日志 | 原因和处理 |
|---|---|
| `open(/dev/ttyACM0) failed: Permission denied` | 当前终端没有串口权限。执行 `id` 检查是否有 `dialout`；没有则执行 `newgrp dialout`，或注销后重新登录，再重新 `source install/setup.bash`。不要用 `chmod 666` 作为永久解决方案。 |
| `open(/dev/ttyACM0) failed: No such file or directory` | 设备不存在、USB 线松动，或插拔后设备号改变。重新执行 `ls -l /dev/ttyACM* /dev/ttyUSB*`，将命令中的路径改成实际路径。 |
| `open(...) failed: Device or resource busy` | 串口被串口调试器、厂商工具、另一个硬件进程或旧进程占用。执行 `fuser -v /dev/ttyACM0` 查找占用者，关闭对应程序后重试。 |
| `tcgetattr failed: ...` | 设备虽然能打开，但不是可配置的串口设备或设备已断开。重新插拔 IMU，确认使用的是正确的 `/dev/ttyACM*`/`/dev/ttyUSB*`。 |
| `tcsetattr failed: ...` | 串口参数配置失败。确认 IMU 串口支持 921600 波特率，并检查 USB 转串口驱动和连接稳定性。 |
| `no serial bytes received` | 串口已打开，但 IMU 没有发送数据。检查 IMU 是否上电、发送功能是否开启、串口线 TX/RX/GND 是否正确，以及波特率是否为 921600。 |
| `received ... raw bytes, but no valid 57-byte frame` | 收到了数据，但帧格式、波特率或数据起始位置不匹配。确认 IMU 输出的是连续三段 19 字节记录组成的 57 字节协议帧，并确认波特率为 921600。 |
| `waiting for a valid 57-byte frame` | 使用的仍可能是旧版本安装程序或旧日志。重新构建并刷新环境：`colcon build --packages-select mymit_robot --symlink-install`，然后 `source install/setup.bash`。 |
| `unable to open IMU serial port` | 这是上面串口错误的汇总信息，不能单独判断原因，查看它前面的 `[DM IMU]` 日志。 |
| `cannot open calibration file` | 完整模式的标定文件路径错误或文件不存在。使用标定文件的绝对路径；首次可先使用 `--imu-only` 验证 IMU。 |
| `motor ID must be in [1,255]` | 标定文件中的电机 ID 仍为 0 或超出范围。检查 `config/dm1_hardware_calibration.txt`，每个 ID 必须是 1 到 255。 |
| `Startup zero rejected` | 完整模式启动零位检查失败。机器人必须保持静止、水平、趴下，并且收到 12 路健康电机反馈；仅测试 IMU 时使用 `--imu-only`。 |
| `DM1 hardware bridge is read-only` | 当前程序默认只读，为安全设计。不要在未确认电机映射和机械零位前添加 `--enable-output`。 |

### 重新构建和环境刷新

修改代码、切换工作空间，或怀疑运行的是旧程序时，执行：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot --symlink-install
source install/setup.bash
hash -r
```

## 构建工程

确认 ROS 2 和相关依赖已安装后，在工作空间中执行：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot
source install/setup.bash
```

## 运行仿真

启动 DM1 仿真（默认自动选择 OpenGL，机器人从趴地零位原地初始化）：

```bash
ros2 run mymit_robot mymit_robot_user --render-gpu auto
<!-- 换模型要修改的地方：当前模型编号 -->
# 启动时选择冻结的 model_4210 RL；不加该参数仍是 MPC/WBC
ros2 run mymit_robot mymit_robot_user --walk-mode rl --render-gpu auto
```

MuJoCo 窗口中的 `Stand up`、`Forward slow`、`Forward fast`、`Backward`、
`Left`、`Right`、`Rotate CCW`、`Use MPC` 和 `Use RL` 按钮可用于站起/回站、
控制器选择和方向控制。启动后先点击 `Stand up`，再点击方向按钮；MPC 与 RL
切换须在停止行走后进行。

在不使用 NVIDIA PRIME 独立显卡渲染的环境中，可以选择自动渲染模式：

```bash
ros2 launch mymit_robot dm1_control.launch.py
```

仅查看不带控制器的 DM1 模型时使用 `dm1_display.launch.py`。

## 更换 RL 模型

<!-- 换模型要修改的地方：当前模型编号 -->
当前平地 RL 模型为 `model_4210.pt`。如果新 checkpoint 与当前模型结构相同
（45 维观测、6 帧历史、12 维动作），只替换模型权重和身份信息，不需要修改
MPC/WBC 行走代码。

首先计算新模型的 SHA256：

```bash
sha256sum /新模型的绝对路径/model_XXXX.pt
```

然后同步修改以下位置：

1. `ros2_ws/mit_robot/CMakeLists.txt` 第 53～88 行：
   - 将 `MYMIT_DM1_RL_CHECKPOINT_DEFAULT` 改为新 checkpoint 的绝对路径。
   - 将 `generated/dm1_policy_4210.hpp` 改为
     `generated/dm1_policy_XXXX.hpp`。
   - 更新对应注释和导出提示；旧模型路径可继续作为注释保留。
2. `ros2_ws/mit_robot/tools/export_dm1_policy_header.py`：
   - 第 37 行 `EXPECTED_SHA256` 改为新模型 SHA256。
   - 第 57、80、82、92 行的模型编号和 `dm1_policy_4210` namespace
     改为新编号。
3. `ros2_ws/mit_robot/include/controller/RlPolicy.hpp` 第 22～30 行：
   - 将 `kDm1FlatCheckpointSha256` 改为新模型 SHA256。
   - 把旧模型名和 SHA 留在注释中，便于回退。
4. `ros2_ws/mit_robot/src/controller/frozen_dwaq_policy.cpp`：
   - 第 8 行改为包含新的 `dm1_policy_XXXX.hpp`。
   - 第 43～132 行将所有 `dm1_policy_4210::` 改为新的 namespace。
5. `ros2_ws/mit_robot/config/dm1_model_contract.yaml` 第 33～34 行：
   - 更新 `flat_checkpoint` 和 `flat_checkpoint_sha256`。
6. `ros2_ws/mit_robot/test/dm1_control_contract_test.cpp`：
   - 更新测试名称、模型身份和 SHA 期望值。

`FrozenDwaqPolicy::metadata()` 与 FSM 白名单当前都使用活动模型名
`model_4210`，并同时校验 SHA256；更换模型时两处必须同步更新。

- `ros2_ws/mit_robot/src/controller/frozen_dwaq_policy.cpp` 第 141～146 行；
- `ros2_ws/mit_robot/src/FSM/FSM_State_Locomotion.cpp` 第 433～436 行。

已有构建目录会缓存旧的 `MYMIT_DM1_RL_CHECKPOINT`。更换模型后，建议在构建时
显式传入新路径，避免继续使用缓存中的旧模型：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DMYMIT_DM1_RL_CHECKPOINT=/新模型的绝对路径/model_XXXX.pt
source install/setup.bash
```

完成后运行契约测试并启动 RL 模式：

```bash
colcon test --packages-select mymit_robot --ctest-args -R test_dm1_contract
colcon test-result --verbose
ros2 run mymit_robot mymit_robot_user --walk-mode rl --render-gpu auto
```

RL 适配器使用训练侧的默认关节参考 `[0, -0.520, 1.330]`，该值已在代码中标注
为“换模型要修改的地方”，必须与新模型训练配置保持一致。

如果新模型的观测维度、历史长度、动作维度、网络层形状、归一化方式或动作缩放
发生变化，就不能只替换权重；还需要同步修改 `RlPolicy.hpp` 中的接口维度、
`export_dm1_policy_header.py` 中的 `EXPECTED` 张量形状，以及
`frozen_dwaq_policy.cpp` 中的网络层和输入拼接逻辑。

## 调整站立速度

硬件版和 MuJoCo 仿真共用同一套站立速度参数。当前已将站立动作调慢：支撑后
机身抬升速率为 `0.04 m/s`，趴地展开阶段为 `1.8 s`。以后调整站立速度时，修改
带有“换站立速度要修改的地方”标记的位置：

- `ros2_ws/mit_robot/include/model/robot_control_parameters.hpp`：修改实际运行参数；
- `ros2_ws/mit_robot/config/dm1_model_contract.yaml`：同步修改契约记录；
- `ros2_ws/mit_robot/src/FSM/FSM_State_StandUp.cpp` 和
  `ros2_ws/mit_robot/user/RobotRunner.cpp`：只需在改变站立控制逻辑时修改，通常不动。

修改后重新构建：

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot --symlink-install
source install/setup.bash
```

## 测试

工程提供了按功能模块划分的测试用例。完成构建后可运行：

```bash
cd ros2_ws
colcon test --packages-select mymit_robot
colcon test-result --verbose
```

## 项目说明

本项目主要用于 DM1 控制算法的开发、验证与学习；新增硬件驱动时应实现
`MitTransport`，不得绕过统一 `JointCommand` 安全边界。

## License

本项目包含采用 Apache-2.0 与 BSD-3-Clause 许可证发布的代码及资源，具体授权信息请参阅工程中的 [LICENSE](ros2_ws/mit_robot/LICENSE) 文件及相关模型目录中的许可说明。
