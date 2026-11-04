# DM1 description

本目录是工程内唯一的 DM1 四足 MuJoCo 模型（`dm1.xml`、`scene.xml` 和 `assets/` 下的
STL 网格），安装到 `share/dm1_robot/description`。模型尺寸、惯量、关节限制和网格来自
`legged_damiao_description`（ROS 1 原包保留在 `third_party/legacy/` 作参考）。

源模型名称映射到控制框架的统一顺序如下：

| 源模型 | 控制框架 | 关节映射 |
| --- | --- | --- |
| RF | FR | HAA / HFE / KFE → hip / thigh / calf |
| LF | FL | HAA / HFE / KFE → hip / thigh / calf |
| RH | RR | HAA / HFE / KFE → hip / thigh / calf |
| LH | RL | HAA / HFE / KFE → hip / thigh / calf |

关节零位与 OpenDog URDF（`config/opendog/dm1_opendog.urdf`）一致，`home` keyframe 是
趴卧姿态 `(0, -0.203, -2.22)`，仿真从这里原地开始而不是从空中落下。物理步长 0.002 s，
与 500 Hz 控制周期一致。修改本目录的质量、惯量、几何或限位时，同步修改 URDF，并运行
`test/model/opendog_model_contract_test.py`。

仅查看模型：`ros2 launch dm1_robot dm1_display.launch.py`。
