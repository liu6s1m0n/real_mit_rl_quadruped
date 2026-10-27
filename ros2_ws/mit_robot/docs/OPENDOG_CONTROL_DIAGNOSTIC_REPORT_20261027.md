# OpenDog 控制诊断报告（2026-10-27）

## 结论

本次分析把启动轮询失败、CAN 缓冲区满和发送失败从控制结论中排除。剩余数据表明，OpenDog 的调度主链已经按原工程的 OCS2 结构运行：状态估计 → `MPC_MRT_Interface` → `SqpMpc` → `WeightedWbc` → MIT 关节命令。当前最明确的控制缺口是：解锁后原来把目标设为 `Passive`，参考轨迹会锁存当前状态，因此没有真正发出回零目标。

本轮已修正为解锁后进入 `JointPd` 目标。该目标保持当前机身状态，并在 `standUpDuration=3.2 s` 内把 12 个模型关节平滑过渡到 DM1 的机械零位（当前标定为 0 rad）。按 `1` 后仍切换到 OpenDog 原有的 `StandUp` 轨迹和 `defaultJointState`。

针对后腿承重时“大腿目标已变化、后小腿仍压地”的现象，已将 OpenDog 直接关节输出从 `kp=4,kd=5` 调整为 `kp=10,kd=2`，后小腿继续使用 `kp=18`。这只调整输出 PD 参数，不改变 OpenDog 的 MPC/WBC 目标和接触调度。

## OpenDog 源码基准

对照 `/home/simon/open-dog/ros2_ws/src/legged_control_adapter/src/TargetTrajectoriesAdapter.cpp`：

- 原工程的目标生成使用两点状态轨迹，时间为当前时刻和目标到达时刻。
- 普通目标轨迹使用 `defaultJointState`，机身目标高度使用 `comHeight`，roll/pitch 归零。
- 原工程的站立轨迹从当前状态开始，在 3.2 s 内把机身高度提升到 `comHeight`，并把关节设置为 `defaultJointState`。
- 原工程的步态调度通过模式模板插入完成，当前项目的 `OpenDogGaitCommand` 保留了这条调用关系。

本次回零只补充了硬件解锁后的初始化边界：它沿用同样的两点 `TargetTrajectories` 形式，不重写 MPC/WBC，也不改变 OpenDog 的 trot 或单步模式调度。

## 日志与 CSV 对比

分析文件位于 `ros2_ws/dm1_diagnostics/real_tests/`。通信字段（`publish_ok`、`tx_failed_frames`）没有参与下面的控制判断。指标使用关节目标误差 `|q-q_des|`、关节速度、总力矩、机身高度和 IMU 加速度。

| 记录 | 控制表现 | 关键特征 | 判断 |
|---|---|---|---|
| `dm1_march_20261027_181640_*.csv` | 稳定 | 最大关节目标误差约 0.019 rad；最大总力矩约 8.8 Nm；机身高度约 0.142–0.146 m；IMU 无突发大峰值 | 可作为当前控制链的正常基线 |
| `dm1_march_20261027_174005_*.csv` | 基本稳定 | 最大关节误差约 0.043 rad；最大总力矩约 10.3 Nm；机身高度逐步上升 | 没有明显外力冲击特征 |
| `dm1_march_20261027_175840_*.csv` | 基本稳定 | 最大关节误差约 0.036 rad；最大总力矩约 10.4 Nm | 没有明显外力冲击特征 |
| `dm1_march_20261027_181539_*.csv` | 有较大偏差 | RL calf 误差约 0.160 rad；IMU 加速度峰值约 1.9/5.4/14.3 m/s² | 有扰动或姿态恢复过程，但尚未出现强冲击 |
| `dm1_march_20261027_181418_*.csv` | 突发异常 | 约 15.19 s，FL calf 误差约 0.548 rad；关节速度峰值约 2.48 rad/s；IMU 峰值约 56/52/100 m/s²；机身高度由约 0.24 m 发生明显变化 | 与手动拉动最吻合：外力先改变机身/关节状态，控制器随后产生大恢复动作 |
| `dm1_march_20261027_181707_*.csv` | 突发异常 | 约 9.21 s，FL calf 误差约 0.538 rad；总力矩约 55.3 Nm；IMU 峰值约 41/41/96 m/s² | 同样是外力/碰撞型冲击，不像调度器周期性失稳 |
| `dm1_march_20261027_171154_*.csv`、`175851_*.csv` | 后段发散 | 约 7–8 s 后 calf 误差约 0.54–0.57 rad，速度约 5.6–7.4 rad/s，总力矩约 59–60 Nm，机身高度接近 0.30 m | 需要重点复现实验；表现为大扰动后的恢复失败或目标/反馈不一致，不能归因于通信字段 |

## 对“手动拉动”那次的判断

手动拉动应对应具有以下组合特征的记录：同一时间窗口内，IMU 三轴加速度突然跃升、机身高度/姿态快速变化、单个或少数关节的 `q` 与 `q_des` 快速分离，然后恢复力矩上升。`181418` 和 `181707` 两段最符合这个模式；它们不是慢慢积累的调度误差。

没有操作者时间标记时，无法仅凭 CSV 给出“哪一段一定是手拉”的绝对证明。`171154` 与 `175851` 的后段发散更像外力后的恢复失败或初始姿态/目标不一致，下一次实验应在按键和手动干预时同步记录时间戳。

## 控制层需要继续验证的点

1. 解锁后先观察 3.2 s 回零轨迹，确认 12 路 `q_des` 从当前值单调过渡到 0 rad，期间不按站立键。
2. 回零完成后再按 `1`，检查 `defaultJointState` 的站立目标是否与 DM1 机械姿态一致。
3. 在不触碰机器的情况下重复三次；若仍出现 `q` 与 `q_des` 持续分离、机身高度自行上升或 calf 速度超过约 2 rad/s，应继续检查 OpenDog 模型关节映射、初始状态和 WBC 输出，而不是重建 CAN 调度器。
4. 若只在手动拉动后出现上述峰值，应增加外力扰动下的恢复策略：限制回零/站立切换期间的关节目标变化率，并在估计到大姿态冲击时暂时保持当前接触模式。

## 验证

- `colcon build --base-paths ros2_ws --build-base ros2_ws/build --install-base ros2_ws/install --packages-select mymit_robot --cmake-args -DMYMIT_ENABLE_OPENDOG=ON`：通过。
- `test_opendog_algorithm`：6/6 通过。测试使用可写的 `ROS_LOG_DIR=/tmp/opendog_ros_log` 和 `ROS_HOME=/tmp/opendog_ros_home`。
- 调参后 `OpenDogMuJoCo.UnfoldsBothRearKneesFromProneHome`、`OpenDogMuJoCo.SingleStepsAndMeasuresPolicyFreshness` 均通过；后膝最终目标误差约 0.012 rad，单步双对角测试通过。
- 构建输出中的告警来自 OpenDog/OCS2 第三方头文件和当前环境缺少实时调度权限，不是本次回零改动引入的编译错误。
