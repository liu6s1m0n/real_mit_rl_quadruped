# 真机算法边界及源码依据

两条控制链各自拥有控制对象、命令状态和算法配置。运行时工厂只构造选中的 runner。

```text
HardwareBridge（反馈、标定、使能、协议校验、CAN发送、日志）
  ├─ current_mit → MitHardwareRunner → CurrentMitBackend → RobotRunner
  │                 → MIT估计器 → ControlFSM → MIT MPC / WBC → LegController
  └─ opendog     → OpenDogRobotRunner → OpenDogAlgorithm
                    → KalmanFilterEstimate → SqpMpc / MPC_MRT_Interface
                    → WeightedWbc → OpenDog关节命令
```

`MitHardwareRunner` 的按键处理、站立等待、安全回退和状态标签从原
`src/hardware/hardware_bridge.cpp` 搬出；控制计算复用 `CurrentMitBackend`。
原 `user/RobotRunner.*` 和 `include/FSM` / `src/FSM` 继续供 MIT 使用，保留仿真接口。

`OpenDogRobotRunner` 的目标、运动序号、按键转换和策略时效日志从原硬件桥搬出。
`DesiredState` 在该 runner 内独立维护，初始化时显式标记有效；高度从已有
OpenDog `opendog_reference.info` 的 `comHeight` 加载。硬件桥不再从 MIT
`RobotRunner::desiredState()` 给 OpenDog 复制目标，也不再为 OpenDog 构造 MIT 控制器。
外部 `ControlInputSnapshot.desired` 不会覆盖 runner 自己的命令状态。

## 对照的 OpenDog 源码

对照目录：`/home/simon/open-dog/ros2_ws/src`，只读。

- `legged_keyboard/src/legged_keyboard.cpp`：前进 `0.18`、后退 `-0.30`、
  横移 `±0.20`、转向 `±0.30`；按 `1` 发布 `/set_stance`，清除
  `trot_requested_`，并发布零速度。新 runner 据此清除锁存运动和速度，
  防止同一控制周期的旧运动覆盖站立请求。
- `legged_control_adapter/src/TargetTrajectoriesAdapter.cpp`：高度来自 `comHeight`，
  速度经姿态旋转到世界坐标系，站立/目标轨迹由该适配器生成。
  本次继续调用工程现有 `OpenDogReference`，没有改写轨迹公式。
- `legged_controllers_ros2/src/LeggedController.cpp`：状态估计、更新策略、
  `evaluatePolicy`、`wbc_->update`，最后以优化状态/输入提取关节位置、速度，
  以 WBC 解的尾部提取力矩。本次保留现有 `OpenDogAlgorithm` 的这条计算链。
- `legged_wbc/src/WeightedWbc.cpp`：继续使用工程内已有 OpenDog WBC 源码，
  不增加 MIT WBC 调用，也不修改 QP 任务或约束。

OpenDog 硬件速度档位单独存放于 `opendog_task.info/hardware_command`，初值沿用
上述原键盘数值；`forward_fast=0.36` 是本工程已有快速档，原 OpenDog 键盘
没有这个档位。本次只隔离它的配置来源，不新增运动算法。

DM1 启动回零在硬件 runner 内执行：移植现有 `RobotRunner::prepareJointInitialization()`
的捕获实测关节角、1.2 s 三次 smoothstep、零前馈计算，终点直接下发 12 路零位置/
零速度并保持。增益读取 OpenDog 自己的 `hardware_output`；不会构造 MIT 控制器。
提前按站立只登记请求，回零轨迹结束后再进入原 OpenDog MPC/WBC。完成提示只表示
轨迹目标到达零，实际回零结果仍以电机反馈为准，不改写电机零点。

旧实现把回零零位当作 MPC 软目标，仍受接触约束优化；2026-10-28 实机 CSV 在
8.33 s 时仍下发最大 0.307 rad 的非零目标。该路径不能承担确定的硬件回零。
这次修正硬件初始化路径，OpenDog 的 `WeightedWbc`、MPC 和轨迹公式均不修改。
回零期间没有基座位置估计器运行，输出的 estimate.valid 为 false。

现有单次踏步保留，它是 DM1 适配行为，并非上游 OpenDog 原样提供。
后膝特殊增益已删除，所有关节恢复原 LeggedController 的 kp=4、kd=5。
首次 MPC 策略遵循原 starting() 的实测状态保持目标，在已有求解线程中准备，
收到初始策略后才开始站立目标轨迹；不增加实际关节必须全部归零的准入条件。
WeightedWbc 仅新增原 QP 求解/取解返回码的诊断存储和访问，不修改数学任务、
约束、迭代次数或输出决策。控制周期保持 2 ms，CSV 计算与写盘移至后台线程。

## FSM 评估

需要隔离 FSM 的所有权和调用，不需要复制一份 MIT `ControlFSM` 给 OpenDog。
MIT 的 FSM 同时持有 MIT 状态、步态和 MPC/WBC 对象；让 OpenDog 使用它会再次耦合。
OpenDog 原控制器通过独立目标轨迹和 OCS2 接触模式调度驱动控制，没有使用
本工程 MIT FSM。本次保持该结构，OpenDog runner 只搬运已有模式命令。

共享的 `ControlMode`、按键事件、反馈/输出数据结构只描述边界数据，
不持有任一算法对象；CAN、IMU、关节标定、电机协议和输出检查属于同一台 DM1
的设备层，继续共享。CSV 足端诊断使用独立只读的 DM1 几何对象，参与日志计算，
不把 MIT 状态估计或足端控制输出注入 OpenDog。

## 构建与验证边界

`mymit_mit_hardware_runner` 和 `mymit_opendog_hardware_runner` 为独立库，
共享硬件桥在运行时选择其一。统一入口仍链接两者以支持 `--algorithm`；
库加载不等于同时构造或运行两套控制器。此拆分针对真机硬件路径，现有仿真入口
及其按键映射没有在本次迁移。

回归测试覆盖 MIT 无需 OpenDog 配置/ROS 上下文启动、OpenDog 目标和速度配置独立、
后退后站立清除旧命令、外部目标不会覆盖 OpenDog runner，以及既有模拟硬件收发路径。
软件构建与 mock 测试不能证明实机站立或踏步稳定。

## 2026-10-28 本次验证

- OpenDog 算法/参考/步态、MuJoCo、模型契约、硬件桥接四组测试通过；
  硬件桥接测试覆盖后台 CSV 的字段一致性和退出时完整写入。
- 原始驱动完整仿真站立测试通过：机身高度约 0.275 m，四个小腿角分别约
  0.729、0.729、0.708、0.708 rad，回零阶段最大位置误差约 0.0324 rad。
- 500 Hz 是原工程设定，不代表当前运行环境始终满足截止时间。完整硬件仿真
  仍观察到约 2.24 ms 的周期中位数和超时警告；未通过放宽周期或隐藏告警宣称解决。
  当前环境仍无法取得 SCHED_FIFO 权限，实机截止时间和起身能力需用新日志复核。
