# OpenDog 算法源码快照

本目录只保留当前 DM1 ROS 2 控制链需要的 OpenDog/OCS2 源文件，不包含
Gazebo、Raisim、Unitree、controller_manager 硬件插件或 CAN 驱动。

源码来源：

- OpenDog 工作空间：`/home/simon/open-dog/ros2_ws/src`
- OpenDog 根仓库提交：`b7d34fbfb2143fd78fe6462ac3941c3f49c5136b`
- OCS2 子仓库提交：`88ce706c17fcc4591b6c24c87a4a85ff6ed9e92d`

移入的算法源文件保持上游实现；当前工程只在 ROS 2 后端边界完成输入、输出、线程和
DM1 关节顺序连接：

- `legged_wbc`：WeightedWbc 及其层级 WBC/约束实现。
- `legged_interface`：LeggedInterface、Pinocchio OCP、约束、初始化器和参考管理。
- `legged_estimation_ros2`：LinearKalmanFilter、StateEstimateBase 和必要的 topic 估计器。
- `ocs2_mpc`：MPC/MRT 基类和接口。
- `ocs2_sqp`：SQP 设置、日志和求解器。
- `hpipm_interface`：原 HPIPM 接口的必要实现。

数值基础库 OCS2 core/oc/qp_solver/centroidal/legged_robot/self_collision/Pinocchio、
HPIPM、BLASFEO、qpOASES 等仍作为构建环境中的数值依赖使用；本目录没有复制与当前
控制链无关的仿真或硬件包。

