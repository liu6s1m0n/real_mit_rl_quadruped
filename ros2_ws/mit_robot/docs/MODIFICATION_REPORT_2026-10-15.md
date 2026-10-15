# 修改报告（2026-10-15）

本报告记录本次提交前的两组改动：第一组来自 Git 暂存区，第二组来自工作区未暂存区。两组改动将分别提交。

## 一、暂存区改动

### 1. 新增按键 5 的单腿动作入口

作用：增加 `SingleLegLift` 命令类型；按键 `5` 进入该命令，速度命令保持为零。

修改前：

```cpp
FourContactLoadTransfer,
};
```

修改后：

```cpp
FourContactLoadTransfer,
SingleLegLift,
};
```

修改前没有按键 `5` 分支；修改后为：

```cpp
} else if (key == '5') {
  command.type = CommandType::Motion;
  command.motion = Motion::SingleLegLift;
} else if (key == 'p' || key == 'P') {
```

速度映射新增：

```cpp
case Motion::SingleLegLift: return {0.0F, 0.0F, 0.0F};
```

文件：

- `include/teleop/operator_command.hpp`
- `src/teleop/operator_command.cpp`

### 2. BalanceStand 的偏航目标跟随当前估计

作用：站立状态不再把偏航目标固定为入场时的值，而是每个控制周期使用当前估计的 yaw。

修改前：

```cpp
wbc_data_.pBody_RPY_des = initial_body_rpy_;
wbc_data_.vBody_Ori_des.setZero();
```

修改后：

```cpp
wbc_data_.pBody_RPY_des = initial_body_rpy_;
wbc_data_.pBody_RPY_des.z() = estimate.rpy.z();
wbc_data_.vBody_Ori_des.setZero();
```

文件：`src/FSM/FSM_State_BalanceStand.cpp`

### 3. Locomotion 识别 THREE_FOOT

作用：将 `THREE_FOOT` 作为独立的站立类入口处理，设置零平移和零偏航速度，并统一浮动基权重。

修改前：

```cpp
const GaitType effective = requested == GaitType::STAND ||
  requested == GaitType::STAND_CYCLE ? requested : GaitType::TROT;
wbc_ctrl_->setFloatingBaseWeight(effective == GaitType::STAND_CYCLE ? T(1000) : T(3));
```

修改后：

```cpp
const GaitType effective = (requested == GaitType::STAND ||
  requested == GaitType::STAND_CYCLE ||
  requested == GaitType::THREE_FOOT) ? requested : GaitType::TROT;
wbc_ctrl_->setFloatingBaseWeight(is_stand_like ? T(100) : T(100));
```

文件：`src/FSM/FSM_State_Locomotion.cpp`

### 4. 键盘帮助和硬件入口

作用：帮助文本增加按键 4、5；硬件控制层将按键 5 映射到 `THREE_FOOT`。

新增帮助文本：

```text
4 一次抬落（FL+RR），5 单腿抬起（FR 腿三腿站立）
```

新增硬件映射：

```cpp
} else if (arbiter.motion() == teleop::Motion::SingleLegLift) {
  gait = GaitType::THREE_FOOT;
}
```

文件：`src/hardware/hardware_bridge.cpp`

### 5. BalanceStand 入场脚底位置缓存

作用：增加入场脚底世界坐标的成员缓存，为后续站立锚定逻辑保留数据结构。

新增代码：

```cpp
std::array<Vec3<T>, kNumLegs> entry_foot_positions_{};
```

该字段目前没有读取逻辑，因此本次提交中不会单独改变运行行为。

文件：`include/FSM/FSM_State_BalanceStand.h`

## 二、未暂存改动

### 1. 按键 4 换载安全门控

作用：阶段一必须连续稳定 50 帧；换载后等待 FR/RL 承重确认；确认前最多卸载 85%，并强制 WBC 保持四脚接触；承重确认连续 50 帧后才打开换载门控并重置 MPC 相位。

新增状态：

```cpp
std::size_t stage1_stable_frames_ = 0;
std::size_t load_transfer_ready_frames_ = 0;
bool load_transfer_gate_open_ = false;
```

新增接口：

```cpp
T estimatedVerticalSupport(std::size_t leg) const;
void updateStage1Stability();
bool loadTransferReady() const;
```

修改前的换载比例：

```cpp
const T blend = phase * phase * (T(3) - T(2) * phase);
```

修改后的门控比例：

```cpp
const bool transfer_waiting = !load_transfer_gate_open_ &&
  load_transfer_elapsed_ >= kTouchdown;
const T guarded_phase = load_transfer_gate_open_ ? phase :
  (transfer_waiting ? T(0) : std::min(phase, T(0.85)));
const T blend = guarded_phase * guarded_phase *
  (T(3) - T(2) * guarded_phase);
```

文件：

- `include/FSM/FSM_State_Locomotion.h`
- `src/FSM/FSM_State_Locomotion.cpp`

### 2. 控制参数调整

作用：提高 roll/pitch 姿态任务比例增益，降低浮动基权重配置。

修改前：

```cpp
Vec3<T>(T(100), T(100), T(100))
T(1000)
```

修改后：

```cpp
Vec3<T>(T(200), T(200), T(100))
T(100)
```

文件：`include/model/robot_control_parameters.hpp`

### 3. 关闭周期性最大电机力矩提示

修改前：每 500 ms 查找最大力矩电机，输出 `NORMAL`、`OVER_CONTINUOUS` 或 `OVER_PEAK`。

修改后：

```cpp
void reportMotorTorque(const dm1_hardware::HardwareSample & sample)
{
  (void)sample;
}
```

故障、发送失败和控制失败的独立告警路径保留。

文件：`src/hardware/hardware_bridge.cpp`

### 4. IMU 持续输出角度

修改前：

```cpp
static_cast<void>(imu.readAt(sample, timestamp));
```

修改后：

```cpp
const bool valid = imu.readAt(sample, timestamp);
const auto yaw_pitch_roll =
  sample.orientation_world_from_body.toRotationMatrix().eulerAngles(2, 1, 0);
```

并每 100 ms 输出：

```text
[DM IMU] ... rpy_deg=(roll, pitch, yaw)
```

角度单位为度；输出姿态是相对于启动静止基准的姿态。原来的四元数输出被移除。

文件：`src/hardware/hardware_main.cpp`

### 5. 测试合同和 Locomotion 入口阈值同步

测试期望值同步为浮动基权重 100、BalanceStand 姿态 Kp `[200, 200, 100]`。

入口稳定阈值修改前后：

```cpp
// 修改前
constexpr float kLocomotionEntryStableDuration = 0.20F;
constexpr float kMaximumEntryRollPitch = 0.15F;

// 修改后
constexpr float kLocomotionEntryStableDuration = 0.10F;
constexpr float kMaximumEntryRollPitch = 0.20F;
```

文件：

- `test/dm1_control_contract_test.cpp`
- `user/RobotRunner.cpp`

## 三、提交顺序

1. 第一笔提交：当前暂存区改动，加上本报告文件。
2. 第二笔提交：第一笔提交后剩余的工作区改动。
