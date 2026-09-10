# DM1 控制与模型说明

硬件、IMU、电机监听和仿真启动命令见：
[RUN_COMMANDS.md](mit_robot/docs/RUN_COMMANDS.md)。

本工作空间只登记 DM1。C++ 模型、MuJoCo XML、RL 输入输出和真实硬件
MIT 接口共享同一份契约，避免以另一机型参数误驱动 DM1。

## 启动

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot
source install/setup.bash
ros2 run mymit_robot mymit_robot_user --render-gpu auto
# 换模型要修改的地方：当前模型编号
# 默认是原有 MPC/WBC；也可启动时选择冻结的 model_4210 RL
ros2 run mymit_robot mymit_robot_user --walk-mode rl --render-gpu auto
```

启动带站立/方向按钮的 MuJoCo 控制窗口：

```bash
ros2 launch mymit_robot dm1_control.launch.py
# 或直接让 launch 以 model_4210 RL 启动
ros2 launch mymit_robot dm1_control.launch.py walk_mode:=rl
```

窗口打开后机器人处于趴地零位且电机默认上锁；先按 `Shift+U` 或点击 `Enable motors`，
再按 `1`/点击 `Stand up`，站起完成后再发送方向命令。方向按键/按钮会锁存当前速度，
按 `Space` 或点击 `Stop` 显式停止。
默认方向按钮进入原有经典 MPC/WBC 行走链。也可以使用 `--walk-mode rl` 启动
model_4210，或在窗口中先点击 `Use RL` / `Use MPC` 再点击方向按钮。切换策略前
先点击 `Stand up` 回到 BalanceStand；行走过程中不会热切换。

只查看 MuJoCo 模型：

```bash
ros2 launch mymit_robot dm1_display.launch.py
```

## 模型契约

契约文件为 `mit_robot/config/dm1_model_contract.yaml`；关节顺序严格为
`FR, FL, RR, RL`，每腿 `hip, thigh, calf`。MuJoCo 和真实电机的 q=0
都是机械趴姿，安装旋转与训练 MJCF 一样烘焙在 body quaternion 中，
解析运动学零偏统一按 `(0, -0.203, -2.22)` 处理；不再把固定旋转重复叠加。

站姿目标为前腿 `[0, -0.597, 1.432]`、后腿 `[0, -0.597, 1.468] rad`，
标称机身高度 `0.39 m`。MuJoCo 执行器峰值范围统一为 `±97 Nm`，仿真桥接层
只执行该配置力矩限幅，不额外叠加速度—力矩降额；真实 MIT 接口仍单独执行
`30 Nm` 连续、`97 Nm` 峰值安全限幅。

## 控制模式

保留 Passive、JointPd、BalanceStand、StandUp、RecoveryStand、经典
MPC/WBC 行走、平地 RL 行走和楼梯 RL 行走。平地 policy 的观测为 45 维、
历史长度 6、50 Hz、动作缩放 0.25；stairs checkpoint 只作为仿真基线，
需要冻结策略白名单后才能请求部署模式。控制器之间必须先回到 BalanceStand，
不能在 Locomotion 内热切换。

`model_4210.pt` 是构建时从
`/home/simon/RL_Robot/logs/trot/dm1_trot_directional_hip_refine_4160/Sep04_17-39-42_directional_hip_from4160_50/`
提取为 C++ 权重的，运行程序不依赖 Python 或 PyTorch；其 SHA256 由 RL 接口
契约校验。上一版 `model_3610.pt` 仅保留为注释回退记录，不参与构建。

## 真机接入

实现 `mit_robot/include/hardware/dm1_mit_interface.hpp` 中的 `MitTransport`
即可连接 CAN 驱动。启动时必须显式加载 12 个电机 ID、方向和零位。反馈由零增益
MIT 帧触发；完整模式启动阶段会先轮询 12 台电机。DM 的 8 字节反馈中 D0 高 4 位
为状态码、低 4 位为物理电机 ID，D6/D7 为 MOS/转子温度；该帧不提供母线电压，
因此电压只在协议明确提供时校验。反馈超时、序号停止增长、非法温度、驱动器故障、
非有限值或超出连续力矩都会关闭全部输出；
没有“默认直连”路径。

完整模式的启动零位策略为：所有关节偏差小于配置窗口（默认 `0.05 rad`）才允许
继续；超出窗口直接拒绝启动。普通启动、`--enable-output` 和键盘解锁都不会写零，
只有显式 `--set-zero` 才执行全电机零位维护。

键盘控制使用独立的电机输出安全闸门：`Shift+U` 解锁、`0` 上锁、`Esc` 上锁并退出，
`W/S/A/D/Q/E` 选择并锁存前后左右和自转，按 `Space` 显式停止。终端 EOF/HUP 只会
禁用终端输入，不会让 MuJoCo GUI 永久进入故障态；仿真 Reset 后重新上锁。完整按键表、
实机命令和安全测试顺序见
[RUN_COMMANDS.md](mit_robot/docs/RUN_COMMANDS.md)。
