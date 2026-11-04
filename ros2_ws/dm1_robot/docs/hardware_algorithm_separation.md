# 真机算法边界及源码依据

工程只保留 OpenDog WBC+NMPC 一套控制算法。MIT Cheetah 控制链（`RobotRunner`、
`ControlFSM`、ConvexMPC、WBIC、`LegController` 及其估计器）已于 2026-11-03 删除；
需要查看旧实现时，回到标签 `savepoint-2026-11-03-opendog-zero`。
"MIT" 在剩余代码中只指达妙电机的 MIT 控制协议（`Dm1MitInterface`、
`PeriodicMitSender`），与控制算法无关。

```text
HardwareBridge（反馈、标定、使能、协议校验、CAN发送、日志）
  └─ OpenDogRobotRunner（按键/目标状态、趴卧初始化、P 趴下）
       → OpenDogAlgorithm
         → KalmanFilterEstimate → SqpMpc / MPC_MRT_Interface
         → WeightedWbc → OpenDog关节命令

SimulationBridge（MuJoCo GUI）
  ├─ 普通仿真：SimImu/SimLeg → OpenDogAlgorithm → SimulationActuatorWriter
  └─ --hardware-sim：HardwareSimulation 设备替身 → 上面完整的 HardwareBridge
```

`OpenDogRobotRunner` 是硬件桥看到的唯一算法边界，由 `HardwareBridge` 直接持有。
`DesiredState` 在该 runner 内独立维护，初始化时
显式标记有效；高度从 OpenDog `opendog_reference.info` 的 `comHeight` 加载。
外部 `ControlInputSnapshot.desired` 不会覆盖 runner 自己的命令状态。

## 对照的 OpenDog 源码

对照目录：`/home/simon/open-dog/ros2_ws/src`，只读。

- `legged_keyboard/src/legged_keyboard.cpp`：前进 `0.18`、后退 `-0.30`、
  横移 `±0.20`、转向 `±0.30`；按 `1` 发布 `/set_stance`，清除
  `trot_requested_`，并发布零速度。runner 据此清除锁存运动和速度，
  防止同一控制周期的旧运动覆盖站立请求。
- `legged_control_adapter/src/TargetTrajectoriesAdapter.cpp`：高度来自 `comHeight`，
  速度经姿态旋转到世界坐标系，站立/目标轨迹由该适配器生成。
  工程调用 `OpenDogReference`，没有改写轨迹公式。
- `legged_controllers_ros2/src/LeggedController.cpp`：状态估计、更新策略、
  `evaluatePolicy`、`wbc_->update`，最后以优化状态/输入提取关节位置、速度，
  以 WBC 解的尾部提取力矩。`OpenDogAlgorithm` 保留这条计算链。
- `legged_wbc/src/WeightedWbc.cpp`：使用工程内的 OpenDog WBC 源码，
  不修改 QP 任务或约束。
- `legged_interface/src/LeggedInterface.cpp`：唯一的算法适配。URDF 足端坐标系是足球
  球心，状态估计也按球心高度 `footRadius`=0.02 m 处理平地；上游摆腿规划以 0 为起落高度、
  支撑约束 `v_z + positionErrorGain·z = 0` 会把支撑脚拉向 0，两者与估计器差 2 cm。
  工程改为以 0.02 m 为起落高度、支撑相只约束速度（2026-11-04 恢复，cb73826 曾撤回）。

OpenDog 硬件速度档位单独存放于 `opendog_task.info/hardware_command`，初值沿用
上述原键盘数值；`forward_fast=0.36` 是本工程的快速档，原 OpenDog 键盘
没有这个档位。

DM1 启动回零在硬件 runner 内执行：捕获实测关节角，1.2 s 三次 smoothstep
插值到趴卧姿态 (0, -0.203, -2.22)，零前馈。增益读取 OpenDog 自己的
`hardware_output`。提前按站立只登记请求，插值结束后再进入 OpenDog MPC/WBC。
完成提示只表示轨迹目标到达趴卧姿态，实际位置仍以电机反馈为准，不改写电机零点。
P 趴下复用同一插值（3 s），前馈从站立时最后一帧 WBC 力矩线性卸载到 0，
完成后复位估计器/MPC/WBC，可再按 1 重新站立。

旧实现把回零目标当作 MPC 软目标，仍受接触约束优化；2026-10-28 实机 CSV 在
8.33 s 时仍下发最大 0.307 rad 的非零目标。该路径不能承担确定的硬件回零，
所以初始化不经过优化器。回零期间没有基座位置估计器运行，输出的
estimate.valid 为 false。

单次踏步是 DM1 适配行为，并非上游 OpenDog 原样提供。
首次 MPC 策略遵循上游 starting()：进入 OpenDog 控制的第一帧同步求解，
拿到初始策略后再交给后台 MPC 线程持续更新。
WeightedWbc 仅新增原 QP 求解/取解返回码的诊断存储和访问，不修改数学任务、
约束、迭代次数或输出决策。控制周期保持 2 ms，CSV 计算与写盘在后台线程。

## 共享设备层

`ControlMode`、按键事件、反馈/输出数据结构只描述边界数据，不持有算法对象；
CAN、IMU、关节标定、电机协议和输出检查属于 DM1 的设备层。CSV 足端诊断使用
`hardware_bridge.cpp` 内独立的 DM1 正运动学（几何取自 `dm1_opendog.urdf`），
只参与日志计算，不进入控制。

## 构建与验证边界

OpenDog 依赖 `/home/simon/open-dog` 下的 OCS2 安装（CMake 变量
`DM1_OPENDOG_PREFIX` / `DM1_OPENDOG_OVERLAY`），找不到时配置阶段直接报错，
不再静默退回其他算法。`sim_main` 与 `hardware_main` 都先链接 ROS 2 的
`tinyxml2`，再加载 MuJoCo，避免两者全局符号冲突。

回归测试覆盖 OpenDog 算法/参考/步态、MuJoCo 站起与单次踏步、URDF 与 MJCF 模型契约、
OpenDog 目标和速度配置独立、后退后站立清除旧命令、外部目标不会覆盖 runner，
以及模拟硬件收发路径。软件构建与 mock 测试不能证明实机站立或踏步稳定。
