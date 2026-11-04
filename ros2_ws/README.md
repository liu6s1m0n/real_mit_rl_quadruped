# DM1 ROS 2 工作空间

工作空间只包含一个包 `dm1_robot`。构建、仿真、真机运行和报错排查见仓库根目录的
[README.md](../README.md)；完整按键表和实机安全顺序见
[RUN_COMMANDS.md](dm1_robot/docs/RUN_COMMANDS.md)。

```bash
cd /home/simon/real_mitrl_dog/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dm1_robot --symlink-install
source install/setup.bash
```

## 模型约定

- 腿顺序 `FR, FL, RR, RL`，每腿关节顺序 `hip, thigh, calf`；数组、标定文件和 CSV 日志
  都按这个顺序。
- MuJoCo 和真实电机的 q=0 都是 OpenDog 机械零位（髋水平、大腿电机接口对准螺丝、
  小腿伸直顶限位），关节角与 `dm1_robot/config/opendog/dm1_opendog.urdf` 一致，
  没有安装零偏；趴卧启动姿态为 `(0, -0.203, -2.22)` rad。
- URDF 与 MJCF 的质量、惯量、几何和限位一致性由
  `dm1_robot/test/model/opendog_model_contract_test.py` 检查。
- 站高目标统一来自 `opendog_reference.info` 的 `comHeight`（真机、`--hardware-sim`
  和普通 MuJoCo 仿真相同）。

## 真机命令边界

`Dm1MitInterface` 是控制命令进入电机前的唯一关口：

- 反馈：12 路地址唯一、序号不回退、驱动器健康、温度 -20～120 °C、电压（协议提供时）
  18～60 V、无故障码，否则本周期丢弃该批反馈、不生成新命令；是否失能由硬件桥决定，
  当前策略是只报错、由独立发送线程保持上一有效命令。
- 命令：非有限值、超出协议范围的速度/增益/电机位置会拒绝整批；关节目标按模型限位
  限幅；预计总力矩超过上限（默认 88 Nm）时按比例缩小该关节的 kp/kd/前馈，其余关节照常发送。
