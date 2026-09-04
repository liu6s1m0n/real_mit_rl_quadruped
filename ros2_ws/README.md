# DM1 控制与模型说明

本工作空间只登记 DM1。C++ 模型、MuJoCo XML、RL 输入输出和真实硬件
MIT 接口共享同一份契约，避免以另一机型参数误驱动 DM1。

## 启动

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mymit_robot
source install/setup.bash
ros2 run mymit_robot mymit_robot_user --render-gpu auto
# 默认是原有 MPC/WBC；也可启动时选择冻结的 model_3485 RL
ros2 run mymit_robot mymit_robot_user --walk-mode rl --render-gpu auto
```

启动带站立/方向按钮的 MuJoCo 控制窗口：

```bash
ros2 launch mymit_robot dm1_control.launch.py
# 或直接让 launch 以 model_3485 RL 启动
ros2 launch mymit_robot dm1_control.launch.py walk_mode:=rl
```

窗口打开后机器人处于趴地零位；先点击 `Stand up`，站起完成后再点击方向按钮。
默认方向按钮进入原有经典 MPC/WBC 行走链。也可以使用 `--walk-mode rl` 启动
model_3485，或在窗口中先点击 `Use RL` / `Use MPC` 再点击方向按钮。切换策略前
先点击 `Stand up` 回到 BalanceStand；行走过程中不会热切换。

只查看 MuJoCo 模型：

```bash
ros2 launch mymit_robot dm1_display.launch.py
```

## 模型契约

契约文件为 `mit_robot/config/dm1_model_contract.yaml`；关节顺序严格为
`FR, FL, RR, RL`，每腿 `hip, thigh, calf`。MuJoCo 和真实电机的 q=0
都是机械趴姿，安装零偏由控制器统一按 `(0, -0.203, -2.25)` 处理；
MJCF 关节参考角与手写运动学保持一致，不再把固定旋转重复叠加。

站姿目标为前腿 `[0, -0.597, 1.432]`、后腿 `[0, -0.597, 1.468] rad`，
标称机身高度 `0.39 m`。MuJoCo 执行器范围为 HAA `±52.4 Nm`、HFE/KFE
`±55 Nm`；真实 MIT 接口仍单独执行 `30 Nm` 连续、`97 Nm` 峰值安全限幅。

## 控制模式

保留 Passive、JointPd、BalanceStand、StandUp、RecoveryStand、经典
MPC/WBC 行走、平地 RL 行走和楼梯 RL 行走。平地 policy 的观测为 45 维、
历史长度 6、50 Hz、动作缩放 0.25；stairs checkpoint 只作为仿真基线，
需要冻结策略白名单后才能请求部署模式。控制器之间必须先回到 BalanceStand，
不能在 Locomotion 内热切换。

`model_3485.pt` 是构建时从
`/home/simon/.codex/worktrees/b9d0/RL_Robot/logs/dm1_trot_hip_refine/`
提取为 C++ 权重的，运行程序不依赖 Python 或 PyTorch；其 SHA256 由 RL 接口
契约校验。原冻结基线 `model_3285.pt` 仅保留为注释回退记录，不参与构建。

## 真机接入

实现 `mit_robot/include/hardware/dm1_mit_interface.hpp` 中的 `MitTransport`
即可连接 CAN 驱动。启动时必须显式加载 12 个电机 ID、方向和零位。反馈
超时、非法温度/电压、驱动器故障、非有限值或超出连续力矩都会关闭全部输出；
没有“默认直连”路径。
