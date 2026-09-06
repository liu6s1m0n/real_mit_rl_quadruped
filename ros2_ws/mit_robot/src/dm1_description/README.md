# DM1 description

本目录是工程内唯一的 DM1 四足模型。模型尺寸、惯量、关节限制和 STL 网格来自：

`~/桌面/legged_examples/legged_damiao/legged_damiao_description`

源模型名称映射到控制框架的统一顺序如下：

| 源模型 | 控制框架 | 关节映射 |
| --- | --- | --- |
| RF | FR | HAA / HFE / KFE → hip / thigh / calf |
| LF | FL | HAA / HFE / KFE → hip / thigh / calf |
| RH | RR | HAA / HFE / KFE → hip / thigh / calf |
| LH | RL | HAA / HFE / KFE → hip / thigh / calf |

运行控制程序；机器人会直接从趴地零位原地初始化，不会从空中掉落。启动后点击
`Stand up`，完成站起后再点击方向按钮：

```bash
ros2 run mymit_robot mymit_robot_user --render-gpu auto
```

MuJoCo 控制面板提供 `Stand up`、`Forward slow`、`Forward fast`、`Backward`、
`Left`、`Right` 和 `Rotate CCW` 按钮。行走中点击 `Stand up` 会回到当前位置的
BalanceStand，不会跳回世界原点。

通过 ROS 2 启动带控制器和方向按钮的窗口：

```bash
ros2 launch mymit_robot dm1_control.launch.py
```

仅查看模型：

```bash
ros2 launch mymit_robot dm1_display.launch.py
```

## 统一模型契约

完整参数见 `config/dm1_model_contract.yaml`。四腿顺序固定为 `FR, FL, RR, RL`，
每腿关节顺序固定为 `hip, thigh, calf`。MuJoCo、RL 和真机都使用电机坐标：
12 个电机零位为全零，机械安装零偏由控制器统一处理为 `(0, -0.203, -2.22)`。

站姿目标为前腿 `[0, -0.597, 1.432]`、后腿 `[0, -0.597, 1.468] rad`，
标称机身高度为 `0.39 m`。
MuJoCo 仿真使用 `0.002 s` 物理步长，平地 RL policy 以 `50 Hz` 运行，
stairs checkpoint 仅作为仿真基线，未标记为可直接部署策略。

真实电机接入使用 `include/hardware/dm1_mit_interface.hpp`：必须显式提供
12 个电机 ID、方向和零位，并在发送前通过反馈超时、温度、电压、驱动器故障
和连续力矩限幅检查。缺失或异常反馈会关闭全部输出。
