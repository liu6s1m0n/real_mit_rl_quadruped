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
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
source /home/simon/open-dog/ros2_ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --packages-select mymit_robot \
  --cmake-target hardware_main --parallel-workers 1 --executor sequential \
  --cmake-args -DMYMIT_ENABLE_OPENDOG=ON

./build/mymit_robot/hardware_main --calibration mit_robot/config/dm1_hardware_calibration.txt \
  --algorithm opendog --algorithm-config mit_robot/config/algorithm
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

构建与运行现统一使用 `ros2_ws/build/`、`ros2_ws/install/`；工程根目录旧产物
移出工作区备份，避免加载不同版本。具体命令见 `RUN_COMMANDS.md`。

## 仍需真机确认的内容

零位、关节方向、安装姿态、CAN 总线名称、反馈/发送时序和控制周期全部沿用当前 MIT
硬件桥与标定文件；OpenDog 不再定义第二套真机设备参数。首次上电仍应先按当前 MIT
流程读取反馈，再单独选择 `opendog` 做站立测试。


## 2026-10-27 OpenDog 单次踏步与数据新鲜度

本轮只修改 OpenDog 的按键请求、接触时序和只读耗时观测。NMPC/WBC 方程、求解器、
状态估计公式、模型、MIT 算法、硬件标定及 CAN 时序没有修改；两个后端继续独立可选。
共享 GLFW 入口补充键 4，使它能够到达各后端已有的命令分支。

### 每轮实现与验证

1. 在 OpenDog 独立 gait 配置增加有限单步模板：键 3 抬左前/右后，键 4 抬右前/左后。
   模板完成后永久 STANCE，不循环。请求序号区分“保持当前按键选择”与“再次按下”。
   在求解线程同步模块中修改接触时序，继续使用原 OCS2 GaitSchedule。
   定向参考/步态测试 4/4：验证正确支撑组、不会自动重踏、重复请求、切换连续小跑、
   停止和重置后的重新触发。
2. MuJoCo 与真机 OpenDog 入口传递请求序号；补齐 GUI 的键 4 和双腿按钮命令。
   新增真实 DM1 MuJoCo 持续测试，使用正式 SimulationActuatorWriter，推进物理
   7000 帧（14 s），包含站起、键 3、键 4、连续小跑、再次键 3。
   检查对应脚抬高、落地后持续四脚支撑、机身高度/姿态及命令有限值，MuJoCo 2/2 通过。
3. 在 OpenDog 外层观测计算、WBC、MPC 耗时和所采用策略的原始观测时间，保留命令的
   输入时间戳；不通过更新到求解结束时间来掩盖延迟。真机只读日志额外使用原始电机
   反馈时间戳计算发布时年龄。算法测试 6/6、硬件桥/独立发送线程测试 31/31、
   遥操作测试 11/11 通过。真实 GUI 的 U→1→3→4→3→Space→Esc 正常退出。
4. 安装产物检查发现原构建目录 RPATH 在安装后丢失，导致 source install/setup.bash
   后缺少 Pinocchio/Coal。为 OpenDog 相关运行目标保留外部链接目录的 INSTALL_RPATH，
   没有变更 MIT GUI 的库顺序。安装库依赖解析正常，安装环境算法 6/6、OpenDog 硬件
   键盘路径 1/1 复测通过。
5. MIT 完整硬件仿真定向回归 CompleteBridgeStandsMarchesAndStops 通过：
   站立 1039 帧、小跑 700 帧、回退 0 次、小跑最小高度 0.278 m。
   本轮没有把此前仍有失败的 MIT 全套行走测试描述为通过。

所有编译通过 colcon，逐一指定目标，CMAKE_BUILD_PARALLEL_LEVEL=1、MAKEFLAGS=-j1、
executor sequential、parallel-workers 1。分别完成 OpenDog GUI、MIT GUI、hardware_main
和相关测试目标；通过 colcon 的 install/fast 目标安装到 `ros2_ws/install`。
部分 colcon 调用在目标生成后没有返回，已中断并重新运行目标，最终复核调用成功完成。
安装时 CMAKE_SKIP_INSTALL_ALL_DEPENDENCY=ON，仅安装已有产物，避免 install 触发全量编译。

### 单实例实测

最终独立性能运行未同时开启其他本项目求解/构建任务，Release -O3，14 s 仿真时间，
7000 帧，墙钟约 14.095 s。当前运行环境未获得 SCHED_FIFO 50 优先级，不能视为
具有硬实时保证的真机测量。以下 compute 使用调用前后的墙钟计时；MPC 一栏是各控制帧
采样到的最近一次完成的求解耗时，并非每次求解等权统计。

| 阶段 | compute 中位数 / P95 (ms) | 最近 MPC 中位数 (ms) | 策略年龄 P95 / 最大 (ms) | 最少剩余时域 (ms) |
| --- | --- | --- | --- | --- |
| 站起/站稳（剔除前 0.1 s） | 0.966 / 1.923 | 91.410 | 180 / 196 | 804 |
| 键 3 | 0.989 / 1.994 | 92.202 | 180 / 196 | 804 |
| 键 4 | 0.959 / 1.856 | 90.922 | 180 / 194 | 806 |
| 连续小跑 | 0.956 / 1.215 | 88.617 | 174 / 186 | 814 |
| 再次键 3 | 0.962 / 1.181 | 90.170 | 178 / 192 | 808 |

首次 compute 为 97.885 ms，内部首次 MPC 为 95.187 ms，首次 WBC 0.726 ms。
其后 6999 帧有 153 帧计算超过 2 ms，最大 2.464 ms；WBC 中位数约 0.31 ms。
这个结果证明当前普通调度环境不能保证每次控制都在 2 ms 内完成。

单次踏步与连续小跑的耗时接近；主要延迟来自约 90 ms 一次的 NMPC 求解，明显达不到
配置中的 mpcDesiredFrequency=100 Hz（10 ms 预算），实际约 10～11 Hz。
没有通过减少迭代、缩短时域、更改离散步长或改线程数来改变算法。

首次独立运行中还观察到 MPC 133.930 ms、策略年龄 258 ms、剩余时域最少 742 ms。
并发开启第二个仿真实例时，GUI 日志出现约 160.6 ms 求解和约 244 ms 策略年龄；
该运行仅用于入口测试，不并入上面的单实例统计。上述短时测试均没有出现预测时域耗尽，
不能据此保证长时、高负载或其他运动条件下不会耗尽。

### 真机新鲜度结论与观测边界

会存在“持续发送但计算结果更新较慢”的情况。CAN 两线程 500 Hz 续发只保证发送频率，
并不代表 NMPC 每 2 ms 更新策略；它仍可能发送上一组控制命令。
控制线程每帧用当前反馈更新状态估计与 WBC，但采用的 NMPC 策略基于较早的观测。

硬件模拟测试已观测到首次发布时 snapshot_age_ms=104.396、原始电机反馈年龄同为
104.396 ms。原发布校验使用控制周期开始的 now_s，compute 阻塞不反映在该次校验中，
所以该帧仍显示 publish_ok=true。本轮保留原发布行为，只增加真实发布时年龄观测，
没有增加丢帧、超时失能、降频、限速或策略拒收。命令时间戳仍保留输入时间。

已有 jointStates() 与桥接层会把部分传感器对象时间戳写成周期时间；新增电机年龄日志
使用 HardwareSample.motors 的原始时间戳，避免把该重标记当成真实采集时间。
IMU 原始采集/传输年龄未在本轮独立验证；真机 CAN、IMU 延迟和长时计算负载仍需实际
运行日志确认。本轮没有连接或驱动真实电机。

单次模板在原有预测时域末端插入，所以按键到抬腿约 1 s，加上同步与求解延迟。
连续小跑切到单次踏步时，已规划前缀仍先执行，再执行一次指定对角步态并四脚站稳。
终端字节没有释放事件，长按导致系统自动重复字节时可产生多个请求；GLFW 仅处理 PRESS。

复现性能测试（先按上面的逐目标方式构建 test_opendog_mujoco）：

```bash
source /opt/ros/humble/setup.bash
source /home/simon/open-dog/ros2_ws/install/setup.bash
source /home/simon/real_mitrl_dog/ros2_ws/install/setup.bash
cd /home/simon/real_mitrl_dog/ros2_ws
export ROS_LOG_DIR=/tmp/opendog_ros_log ROS_HOME=/tmp/opendog_ros_home
export OPENDOG_TIMING_CSV=/tmp/opendog_freshness.csv
build/mymit_robot/test_opendog_mujoco \
  --gtest_filter=OpenDogMuJoCo.SingleStepsAndMeasuresPolicyFreshness
```

CSV 包含逐帧 compute/WBC/MPC、策略年龄、剩余时域、墙钟、模式和四脚高度。
本轮最终单实例原始数据为 /tmp/opendog_freshness_isolated.csv，统计为
/tmp/opendog_freshness_summary.json；GUI 为 /tmp/opendog_keys_gui_check.log。


## 2026-10-27 统一 ROS2 构建目录

MIT 与 OpenDog 均在 `/home/simon/real_mitrl_dog/ros2_ws` 内通过 colcon 构建，
共用 `build/`、`install/`、`log/`。本轮只调整运行文档和采集脚本的构建提示，
算法、模型、硬件零位及 CAN 时序没有修改。根目录旧构建产物移至 `/tmp/real_mitrl_dog-root-colcon-20261027-171114`。

- 硬件、MIT GUI、OpenDog GUI、高度服务逐目标单线程构建并安装成功。
- 新终端加载 `ros2_ws/install/setup.bash` 后，ROS2 包定位、四个程序的链接目标及
  动态库依赖解析通过，setup 链不再引用根目录 install。
- 两个 GUI 的 `ros2 run --help` 通过；硬件入口对两种算法参数均接受，未提供
  标定时在设备访问前退出，没有连接真实电机。
- 使用安装库运行 OpenDog 算法/参考/步态测试 6/6 通过。
- MIT 完整硬件仿真站立/踏步/停止定向测试 1/1 通过：stand=1039、trot=700、
  fallback=0、min_trot_z=0.278 m。
- 33 段文档 Bash 命令与采集脚本通过语法检查。colcon 有目标生成后未退出的
  调用，已中断并单独复核；最终各目标和安装调用均以 0 返回。

## 2026-10-27 OpenDog 真机后小腿展开排查

用户确认同一台机器的 MIT 能正常展开两条后小腿。已核对当前腿序到 Pinocchio
腿序的映射、共用硬件接口和标定；模型质量、惯量、关节安装/轴向/范围检查 3/3
通过。现有真机日志中，后小腿存在目标与反馈偏差，原 kp=4 的位置反馈力矩约
0.4～0.5 Nm。尚未证明唯一根因；原参数在当前 MuJoCo 和原始驱动闭环仿真中
也能站起，因此不能把仿真通过作为真机故障已解决的依据。

本轮仅增加 OpenDog 输出参数 `hardware_output.rear_calf_kp=18`，取值参考当前
MIT 的小腿位置增益。仅 RR/RL 小腿使用此值，前腿和其他关节仍为 kp=4，所有
关节 kd=5。不修改 WBC/NMPC 求解、机械模型、零位、安装位置、CAN 时序或 MIT。
旧配置缺少该参数时使用已有 kp；启动日志输出实际加载值。本参数属于真机
位置跟踪验证调整，实机效果仍需重新运行确认。本轮没有驱动真实电机。

本轮测试与评估：

- OpenDog 算法/参考/步态测试 6/6，通过后小腿和其余关节实际输出增益检查。
- MuJoCo 当前场景反馈、趴卧后站起通过：高度 0.2769 m，后小腿约 0.718 rad。
- 单次踏步首轮未全部通过：第二次单步直到 8.010 s 才结束，超过测试 8.000 s
  的四脚支撑检查点，出现 5 帧时序断言失败。该轮最近一次 MPC 耗时最大
  230.430 ms、策略年龄最大 410 ms。没有放宽测试或修改算法。
- 单次踏步独立复测通过：第二次动作在 7.918 s 结束，最近一次 MPC 耗时最大
  142.417 ms、策略年龄最大 252 ms。说明异步求解时序存在运行间波动；不能把
  首轮失败抹去，也不能声称所有负载下均满足固定动作完成时间。
- 共用 HardwareBridge、原始 CAN/串口 IMU 驱动的 OpenDog 站起测试通过：
  高度 0.2772 m，后小腿 0.7212/0.7218 rad。
- MIT 完整硬件仿真站立/踏步/停止定向回归通过：stand=1039、trot=700、
  fallback=0、min_trot_z=0.278 m。
- 删除新增参数的旧配置兼容测试通过，启动日志确认 rear_calf_kp=4；
  高度 0.2747 m，两条后小腿约 0.703 rad。

新增站起测试包含实际 DM1 趴卧初始姿态；完整硬件测试覆盖 U→1→退出、
协议量化和驱动线程。编译继续只在 ros2_ws 内逐目标单线程使用 colcon。
原始记录位于 `/tmp/opendog_rear_fix_test_*.log`，首轮与独立复测逐帧数据分别为
`/tmp/opendog_freshness.csv`、`/tmp/opendog_rear_fix_freshness_isolated.csv`。

已逐目标完成硬件入口及两个仿真入口构建，并经 colcon install/fast 安装到
ros2_ws/install；目标生成后挂起的调用已超时退出，增量重试最终均以 0 返回。
新终端环境的包路径、三个入口的动态库解析、两种算法参数解析通过。
使用安装库复测 OpenDog 算法/参考/步态 6/6，确认加载 rear_calf_kp=18。
工程根目录没有重新生成 build/install/log。
