# DM1 双算法移植状态

## 已实现的运行结构

`HardwareBridge` 每个周期只调用启动时选择的一个后端。两个后端接收同一份
`ControlInputSnapshot`，输出同一个 `ControlOutput`，最后由同一条 DM1 硬件写入路径
发送位置、速度、Kp、Kd 和前馈力矩。

- `current_mit`：保留当前 RobotRunner、FSM、凸 MPC、KinWBC/WBIC 数学实现。
- `opendog`：在本包内编译移入的 LeggedInterface、KalmanFilterEstimate、WeightedWbc，
  使用 OCS2 SQP/MPC 数值库和当前 DM1 机械模型。
- 不支持运行中热切换；启动参数决定本次进程使用的后端，未指定时为 `current_mit`。
- OpenDog 不直接访问 CAN、controller_manager、Gazebo 或仿真控制插件。
- MuJoCo 原生仿真也支持同一选择：`current_mit` 保留原 `RobotRunner` 路径，`opendog`
  直接读取当前 DM1 场景的 MuJoCo IMU/关节反馈并写回同一个执行器路径。
- 为保持原 MIT MuJoCo 的动态库顺序，OpenDog 仿真由同目录的
  `mymit_opendog_user` 进程承载；原入口收到 `--algorithm opendog` 后自动转交。

## OpenDog 源码范围

`opendog/` 只包含当前控制链需要的源文件：

- `legged_interface`：腿式 OCP、Pinocchio 模型和 gait/reference 管理。
- `legged_estimation_ros2`：线性 Kalman 估计器。
- `legged_wbc`：加权 WBC。
- `ocs2_mpc`、`ocs2_sqp`、`hpipm_interface`：本地使用的 MPC/SQP/HPIPM 接口源。

OCS2 的通用数值库、Pinocchio、HPIPM、BLASFEO 和 qpOASES 作为外部数值依赖链接；没有
链接 OpenDog 的 `legged_*` 预编译算法库。源代码来源和提交记录见 `opendog/README.md`。

## 当前 DM1 配置

参数集中在 `config/algorithm/`：

- `dm1_opendog.urdf`：按当前 `src/dm1_description/dm1/dm1.xml` 的质量、惯量、关节轴、
  安装旋转和限位生成，不使用 OpenDog 原机械尺寸。
- `opendog_task.info`、`opendog_reference.info`、`opendog_gait.info`：算法任务、参考轨迹
  和步态参数。
- `opendog.yaml`：选择后端和四个配置文件路径。路径字段会在 OpenDog 初始化时读取。
- DM1 当前顺序 `FR, FL, RR, RL` 到 OpenDog 的 `RF, LF, RH, LH` 使用代码中的固定排列
  `{2, 0, 3, 1}`，随当前机械设定维护，不伪装成未读取的 YAML 参数。
- 真机控制周期由当前 MIT `control_config::control_time_step` 传入 OpenDog；OpenDog 任务文件
  不定义第二套硬件周期。

命令行示例：

```bash
source /opt/ros/humble/setup.bash
source /home/simon/open-dog/ros2_ws/install/setup.bash
colcon build --base-paths ros2_ws --packages-select mymit_robot \
  --cmake-target hardware_main --parallel-workers 1 --executor sequential \
  --cmake-args -DMYMIT_ENABLE_OPENDOG=ON

./build/mymit_robot/hardware_main --calibration CONFIG/dm1_hardware_calibration.txt \
  --algorithm opendog --algorithm-config ros2_ws/mit_robot/config/algorithm
```

也可以把 `opendog.yaml` 的 `algorithm.active_backend` 设为 `opendog`，省略
`--algorithm`；显式命令行参数优先。

MuJoCo 选择 OpenDog：

```bash
ros2 run mymit_robot mymit_robot_user --algorithm opendog --render-gpu auto
```

原有 MIT 仿真命令保持不变：

```bash
ros2 run mymit_robot mymit_robot_user --algorithm current_mit --render-gpu auto
```

## 已验证项目

- OpenDog 后端独立测试：Kalman → NMPC → WBC → 12 路有限命令，包含停止/重启。
- 公共参考轨迹和原 gait synchronized insertion 测试。
- 当前 DM1 URDF 质量、逐连杆惯量、关节 frame/axis/limit 契约测试。
- MuJoCo 与 ROS2 同进程的系统 `tinyxml2` 符号顺序测试。
- `HardwareBridgeTest.OpenDogKeyboardUsesSharedHardwareOutput`：键盘命令经过公共快照、
  OpenDog 后端和公共硬件输出，未启用另一套 MIT 算法。
- `OpenDogMuJoCo.UsesCurrentDm1SceneFeedback`：直接加载当前 `dm1/scene.xml`，将 MuJoCo
  IMU 和四腿关节反馈送入 OpenDog，并验证 WBC+NMPC 产生 12 路有限命令。

每轮验证使用单目标、串行 `colcon build`，没有使用独立 CMake 构建命令。

## 2026-10-27 MIT 仿真回归复查

用户报告前进、后退时摆腿过快。本轮恢复 MIT 的 RobotRunner 在 `simulation.Load`
之前初始化的顺序，并恢复原可执行文件的 MuJoCo 依赖顺序。OpenDog 选择仍由原入口
转交 `mymit_opendog_user`；其 ROS2/tinyxml2 依赖顺序独立配置。没有修改 MIT 的步态、
MPC/WBC、关节控制公式、机械模型或 `control_config.h`。

对照保留在 `ros2_ws/build/mymit_robot` 中、没有 OpenDog 依赖的原程序，在真实桌面窗口
分别执行解锁、站起、前进/后退和停止，并通过临时只读观测库记录 `mj_step` 后的状态。
观测代码位于 `/tmp/mit_original_bridge_audit`，没有加入运行程序。

- 两套程序的模型步长均为 0.002 s，仿真时间/墙钟时间约为 1.000。
- 前进各采样点的机身位置、关节速度和力矩相同；采样到的最大关节速度均为
  4.6386 rad/s。该值是每 0.5 s 采样一次得到的最大值，不代表全程瞬时峰值。
- 后退测试的解锁/运动按键存在墙钟时序差异，轨迹不能逐点视为相同；当前程序的
  采样关节速度没有高于原程序，尚未复现用户报告的摆腿加速。
- 用原版执行器源码和带完整 OpenDog 依赖的执行器分别运行 MIT 行走回归，均为
  4 项通过、相同的 6 项失败，128 行数值诊断相同。不能将该组测试描述为全部通过。
- MIT 模型测试 6/6、OpenDog MuJoCo 测试 1/1、OpenDog 共享硬件桥接测试 1/1 通过。
- 完整硬件仿真定向测试 `CompleteBridgeStandsMarchesAndStops` 通过：站立 1039 帧、
  TROT 700 帧、回退 0 次，测试中的最小小跑高度 0.278 m。

构建与运行统一使用工程根目录 `build/`、`install/`；原先文档指向的
`ros2_ws/install/` 仍保留旧产物。具体命令见 `RUN_COMMANDS.md`。

## 仍需真机确认的内容

零位、关节方向、安装姿态、CAN 总线名称、反馈/发送时序和控制周期全部沿用当前 MIT
硬件桥与标定文件；OpenDog 不再定义第二套真机设备参数。首次上电仍应先按当前 MIT
流程读取反馈，再单独选择 `opendog` 做站立测试。
