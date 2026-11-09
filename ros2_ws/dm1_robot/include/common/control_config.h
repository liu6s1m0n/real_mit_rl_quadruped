/**
 * @file control_config.h
 * @brief 软件算法控制参数的唯一修改入口；修改后重新编译。
 *
 * OpenDog 的 MPC/WBC/状态估计参数在 config/opendog 下的 .info 文件中；这里只保留
 * 硬件链路、遥操作档位、站高和姿态等与算法实现无关的公共参数。
 * 向量顺序：关节 [hip, thigh, calf]；四腿顺序为 [FR,FL,RR,RL]。
 */
#ifndef DM1_ROBOT_COMMON_CONTROL_CONFIG_H_
#define DM1_ROBOT_COMMON_CONTROL_CONFIG_H_

#include <array>
#include <cmath>
#include <cstddef>
#include "common/robot_types.hpp"

namespace control_config
{
/// 公共时序参数；站高统一取 opendog_reference.info 的 comHeight。
  inline constexpr double control_time_step = 0.002;  ///< 控制周期，500 Hz。

/// 操作档位：键盘、界面和硬件遥控使用同一组速度大小，具体方向由命令决定。
  namespace motion
  {
    inline constexpr float forward = 0.18F;  ///< 选择慢速前进档时的目标前进速度；增大后同一慢档命令要求走得更快。
    inline constexpr float forward_fast = 0.36F;  ///< 选择快速前进档时的目标前进速度；应大于慢档。
    inline constexpr float backward = 0.18F;  ///< 后退档的速度大小；下游自动添加后退方向的负号，这里填写正数。
    inline constexpr float lateral = 0.20F;  ///< 左移和右移档共用的横移速度大小；左右方向由命令选择。
    inline constexpr float yaw_rate = 0.30F;  ///< 左右转向档共用的转向速度大小；增大后要求更快地改变朝向。
    inline constexpr int watchdog_ms = 500;  ///< 开启 watchdog 的运动命令多久未刷新后自动停止；当前键盘/GUI 是锁存命令，默认不启用此超时。
  }

/// DM1 关节姿态（模型坐标 = 电机坐标 = OpenDog URDF 关节角，单位 rad）。
/// 电机固件零位 = OpenDog 机械零位：髋水平；大腿电机接口对准螺丝（大腿水平朝后）；
/// 小腿向后伸直顶到限位。该姿态脚在身体后方，只用于悬空写零，不是启动姿态。
  namespace pose
  {
    inline constexpr double prone_hip = 0.0;  ///< 趴卧（启动/趴下保持）髋关节角。
    inline constexpr double prone_thigh = -0.203;  ///< 趴卧大腿角；相对 OpenDog 零位向下 0.203 rad。
    inline constexpr double prone_calf = -2.22;  ///< 趴卧小腿角；小腿向前折叠，离折叠限位 -2.25 约 0.03 rad。
  }

/// 硬件链路的软件默认值：启动检查、反馈时效、调度告警与重连策略。
  namespace hardware
  {
    inline constexpr float torque_limit = 88;  ///< 硬件发送链和仿真执行端默认的每关节力矩上限；启动参数可显式覆盖。
    inline constexpr double feedback_timeout = 0.10;  ///< MIT 接口判定电机反馈是否过期的时限；增大允许使用更旧的数据，不会增加反馈更新频率。
    inline constexpr float startup_stationary_velocity = 0.05F;  ///< 启动检查允许的关节速度绝对值；减小后更严格地要求电机静止。
    inline constexpr int maximum_consecutive_overruns = 3;  ///< 连续控制周期超时达到多少次时触发超时处理/告警；当前处理会丢弃过期周期，不自动失能。
    inline constexpr int maximum_overruns_in_window = 5;  ///< 滑动统计窗口内累计超时达到多少次时触发超时处理/告警；与连续次数条件任一满足即可。
    inline constexpr double overrun_window = 1;  ///< 统计零散控制超时的时间窗口；相同次数阈值下，增大窗口更容易累积触发。
    inline constexpr unsigned int motor_frame_divider = 4;  ///< 未启用控制输出时，多少个控制周期做一次电机反馈轮询；增大可减轻总线负担，不改变独立 MIT 续发频率。
    inline constexpr float startup_maximum_angular_speed = 0.15F;  ///< 启动检查允许的机身总角速度；超过则认为机器人仍在转动。
    inline constexpr float startup_gravity_minimum = 7;  ///< 启动时加速度模长的有效下界；用于排除明显非静止或异常 IMU 样本。
    inline constexpr float startup_gravity_maximum = 12;  ///< 启动时加速度模长的有效上界；与下界共同判断样本是否接近正常重力。
    inline constexpr float startup_maximum_tilt = 0.25F;  ///< 启动检查允许的机身倾斜；根据横向重力分量判断，增大后允许更倾斜地启动。
    inline constexpr double imu_loss_before_recovery = 0.5;  ///< IMU 连续读取失败达到多久才尝试重新连接；增大后更容忍短时抖动。
    inline constexpr double imu_recovery_interval = 2;  ///< 两次 IMU 重连尝试之间的最小间隔；增大后重试更慢，减少反复打开串口。
  }

} // namespace control_config

#endif // DM1_ROBOT_COMMON_CONTROL_CONFIG_H_
