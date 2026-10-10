# WBC 前馈统一为 1.0 倍

日期：2026-10-10。未连接实机或发送实机电机指令。

## 修改

生产代码仅修改 `ros2_ws/mit_robot/src/FSM/FSM_State_BalanceStand.cpp`：
删除唯一的 WBC 前馈乘 1.3 操作，直接使用原始求解结果，并更新日志。
起立的 WBC 抬升阶段、平衡站立、站立调高和停步回站全部复用这个状态。
经典行走及主动趴下原本就是 1.0 倍；RL/关节 PD 模式的显式前馈仍为零。
未增加模式过滤或重新加入已移除的 100 ms 入场混合，公共接口未改变。

回归测试新增 `Dm1Contract.BalanceStandUsesRawWbcFeedforward`，逐腿比较
站立状态前馈与同输入、同配置的实际 WBC 原始结果。
测试准备修正后，在 1.0 倍实现下通过，在离线加载的旧 1.3 倍实现下失败。

## 验证结果

模型契约 16/16、原地踏步 3/3、硬件反馈路径对照 2/2、完整硬件算法链仿真
4/4、MPC 行走 4/5，共 29/30 项通过。完整硬件算法链正常场景最低行走高度
0.308 m，进入步态 700 帧，无 FSM 回退，能够停回站立。

未通过项：`Dm1MpcWalkGate.LocomotionWaitsUntilTheBodyHasStoodUp`。
它正常进入行走，入场和最低行走高度均约 0.307 m；2 s 前进位移为
0.04626 m，未达到原测试 >0.05 m 的阈值。测试阈值未放宽。
离线加载旧 1.3 倍实现，对应位移约 0.053 m，说明本次倍率调整会影响该场景的推进。

该提前请求行走场景，使用同一关节测量计算相邻帧的 FF/PD 命令差：

| 首个行走 WBC 输出帧 | 旧 1.3 倍 | 新 1.0 倍 |
|---|---:|---:|
| 十二关节最大前馈跳变 N·m | 4.8526 | 2.3504 |
| 十二关节最大 FF+P+D 跳变 N·m | 3.0120 | 2.9407 |

因此倍率差造成的前馈跳变已经减小，但总力矩连续性仍受 WBC 目标、配置和
关节 PD 影响。不能将这次倍率统一视为已经解决实机倒地或全部过渡问题。

日志：`/tmp/dm1_wbc_unity_build.log`、`/tmp/dm1_wbc_unity_tests.log`、
`/tmp/dm1_wbc_unity_contract_final.log`；切换探针结果在
`/tmp/dm1_unity_entry_probe_scaled.log` 和 `/tmp/dm1_unity_entry_probe_unity.log`。
