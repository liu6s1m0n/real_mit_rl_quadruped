# DM1 实机验证摘要（2026-10-09 09:39，Asia/Shanghai）

## 测试环境

- Git HEAD：`123a4247a8a02b2372cd3f64d91b112f81ff4e3b`
- 控制参数未提交差异 SHA-256：`e7fe31e788697844bff3bf05684fc347ad58c49ce598da14116fb0ed1d7c28c0`
- 标定文件：`ros2_ws/mit_robot/config/dm1_hardware_calibration.txt`
- IMU：`/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00`
- CAN0/CAN1：CAN-FD，仲裁段 1 Mbps，数据段 5 Mbps
- 软件总力矩限制：30 Nm
- 测试顺序：锁定启动 -> `U` 使能 -> `1` 站立 -> `2` 原地小跑 -> 安全回退 -> `0`/Esc 失能退出

## 结果

### 启动与站立

- IMU 启动基线通过，重力模长 10.092 m/s²。
- 12 路反馈首轮检查通过。
- 使能后发生 1 次边界校验拒绝：FL_calf 目标/实测过渡值 -0.0312548 rad，模型下限 -0.0300000 rad；系统保持上一有效命令，没有失能或断流。
- 站立请求成功，FSM 稳定在 `BALANCE_STAND`。
- WBC 周期约 0.36--0.42 ms，预算 2 ms，记录到的超时次数为 0。
- 站立/测试全程最大反馈力矩绝对值 14.51 Nm（RL_calf），低于 30 Nm 连续额定值。

### 原地小跑

- Locomotion 成功开始，WBC 显示 4 个任务、2 个接触腿。
- 初期 RL_calf 原始总力矩出现 36.355--38.620 Nm，软件和硬件两层均限制到 30 Nm。
- 随后 FL 足端进入无效区域：`p=(0.1062,0.1083,0.0157) m`，`q=(0.6096,-0.4788,0.1007) rad`。
- Locomotion 安全逻辑正确回退到 BalanceStand，并取消运动请求，没有自动重新进入步态。
- 回退后 FL_calf 保持在约 0.1007 rad、RL_calf 保持在约 0.0658 rad，而目标为 1.0040 rad；两关节没有恢复跟踪。
- 最大约束前总力矩为 132.312 Nm（RL_calf，`q_des=1.0040`、`q=0.0658`），实际下发限制为 30 Nm。
- 日志中共记录 123 条算法层总力矩约束信息、34 条硬件层力矩截断信息、1 次安全回退。
- 该现象表明主要故障是关节没有产生与位置误差/命令相匹配的机械运动或反馈力矩，Kp/Kd 只是把持续存在的位置误差放大，并非首要根因。

## 总线状态（停机后）

- CAN0：ERROR-ACTIVE，bus-errors=0，error-pass=0，bus-off=0；RX 111749，TX 56126，TX dropped 10。
- CAN1：ERROR-ACTIVE，bus-errors=0，error-pass=0，bus-off=0；RX 111759，TX 56138，TX dropped 9。
- 停机后确认不存在 `hardware_main`/`ros2 run mymit_robot` 进程，电机已失能。

## 结论与下一步

1. 站立控制和 30 Nm 保护有效；通信链路没有 CAN bus-off 或接收错误。
2. 当前不应再次尝试行走。需要逐电机零增益轮询或单电机低增益测试，优先检查 FL_calf、RL_calf，同时核查 FR/RR 在步态切换后的关节位置。
3. 应检查电机使能状态、供电压降、驱动器故障码、减速器/连杆是否卡滞，以及模型角度与电机方向映射；仅继续降低 Kp/Kd 不能解决该问题。
4. FL_calf 的 -0.03125 rad 一次性边界拒绝需要单独修正量化容差，但它不是本次行走回退的主要原因。

## 原始数据完整性

- 原始日志：`hardware_session.log`
- 原始日志行数：384
- 原始日志大小：50671 bytes
- 原始日志 SHA-256：`c4045e0919b34756eebfa6ce93ddd60a94796f8b2a1e6527de711b3af4322f28`

