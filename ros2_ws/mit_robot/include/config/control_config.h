/**
 * @file control_config.h
 * @brief 软件算法控制参数的唯一修改入口；修改后重新编译。
 *
 * 关节向量 [hip, thigh, calf]；机身/足端向量 [x,y,z]；姿态 [roll,pitch,yaw]；
 * 四腿 [FR,FL,RR,RL] = [右前,左前,右后,左后]。MPC Q 的姿态顺序单独为 [yaw,pitch,roll]。
 *
 * 调参阅读方式：注释写明作用对象、何时生效及主要影响。带“承接”的字段只负责传值；
 * 带“通用”的默认值用于独立任务/控制器，完整运动通常使用文末 RobotControlParameters。
 * Kp 表示偏离目标后纠正的强度，Kd 表示对速度偏差的阻尼；二者不是动作速度旋钮。
 * 调大增益/权重不保证实际动作单调变快或更稳，最终结果还受任务优先级和力矩/接触约束影响。
 * 下游只转换类型、单位或传递参数，不重新指定数值。0/1 状态值、数学公式中的
 * 系数和物理模型/协议常量不是调参项。机械标定及 MuJoCo 参数仍在原模型文件。
 */
#ifndef MYMIT_ROBOT_CONFIG_CONTROL_CONFIG_H_
#define MYMIT_ROBOT_CONFIG_CONTROL_CONFIG_H_

#include <array>
#include <cmath>
#include <cstddef>
#include "model/robot_types.hpp"

namespace control_config
{
// 公共时序、站高和接触约束：MPC/WBC 共用同一来源。
inline constexpr double control_time_step = 0.002; // 硬件控制循环的默认间隔，也是部分控制器的默认时间基准；减小会提高更新频率和计算负担。
inline constexpr double gravity = 9.81; // 算法计算重力补偿、预测运动和名义支撑力时使用的重力值；各控制模块共用。
inline constexpr double friction = 0.4; // MPC/WBC 的地面摩擦假设；越大越允许脚施加水平力，设得过大会高估地面的防滑能力。
inline constexpr double minimum_normal_force = 0; // 支撑脚向上支撑力的下限；增大后不允许支撑脚轻载，摆动脚仍按无接触处理。
inline constexpr double maximum_normal_force = 120; // 每只支撑脚的支撑力上限；增大可分配更大支撑力，但不会直接提高电机力矩限幅。
inline constexpr bool use_wbc = true; // 行走状态是否用 WBC 把 MPC 结果转成关节命令；关闭时走足端控制分支，不影响站立/趴下的 WBC。
inline constexpr double minimum_standing_height = 0.28; // 外部站立高度命令的下限；同时供界面、服务和控制入口校验使用。
inline constexpr double maximum_standing_height = 0.42; // 外部站立高度命令的上限；提高它只放宽可请求范围，不会自动提高当前目标。
inline constexpr double standing_height = minimum_standing_height; // 启动时采用的站立高度目标；默认跟随高度下限，运行时仍可由高度命令调整。

// 操作档位：键盘、界面和硬件遥控使用同一组速度大小，具体方向由命令决定。
namespace motion
{
inline constexpr float forward = 0.18F; // 选择慢速前进档时的目标前进速度；增大后同一慢档命令要求走得更快。
inline constexpr float forward_fast = 0.36F; // 选择快速前进档时的目标前进速度；应大于慢档，实际响应还受 MPC 速度滤波影响。
inline constexpr float backward = 0.30F; // 后退档的速度大小；下游自动添加后退方向的负号，这里填写正数。
inline constexpr float lateral = 0.20F; // 左移和右移档共用的横移速度大小；左右方向由命令选择。
inline constexpr float yaw_rate = 0.30F; // 左右转向档共用的转向速度大小；增大后要求更快地改变朝向。
inline constexpr int watchdog_ms = 500; // 开启 watchdog 的运动命令多久未刷新后自动停止；当前键盘/GUI 是锁存命令，默认不启用此超时。
inline constexpr float maximum_preset_speed = 0.6F; // 仿真界面速度档位设置接口允许的上限；仅校验档位设置，不代替 MPC 的速度限制。
inline constexpr float maximum_preset_yaw_rate = 1.5F; // 仿真界面转向档位设置接口允许的上限；实际 MPC 命令另受 maximum_yaw_rate 校验。
}

// MPC：预测机身运动并分配各脚支撑力；权重决定“更看重哪项误差”。
namespace mpc
{
inline constexpr std::size_t horizon = 10; // 预测未来的段数，须为 2..18 的偶数；增大会扩大预测窗和求解量，也会加长由它派生的 TROT 周期。
inline constexpr double time_step = 0.030; // 每个预测段的时长，也决定 MPC 求解间隔；按控制帧取整后，TROT 周期 = horizon × 实际段长。
inline constexpr double alpha = 4e-5; // 接触力大小的惩罚权重；越大越倾向少用力，越小越优先追踪机身目标。
inline constexpr double x_drag = 0; // 预测模型中世界 x 方向速度对竖直加速度的耦合补偿；0 表示不补偿，不是普通的前进速度阻尼。
inline constexpr std::array<double, 12> state_weights{ // 各状态跟踪误差的惩罚权重；越大越重视该项，0 表示不直接惩罚该项误差。
  0, 0.25, 10, 2, 2, 50, 0, 0, 0, 0.2, 0.2, 0.1};
// Q 顺序：[偏航,俯仰,侧倾,前后位置,横向位置,高度,世界角速度x/y/z,世界线速度x/y/z]。
// 例如加大高度项会更重视保高，加大速度项会更重视走到目标速度；各项不是直接电机增益。
inline constexpr std::array<double, 3> com_offset_body{0, 0, 0}; // 算法认为的重心相对机身原点的位置偏移；会改变力臂和前后/左右支撑分配，不修改机械模型。
inline constexpr double maximum_com_offset = 0.05; // 允许输入的重心偏移总长度上限；只用于拒绝过大的标定值，不直接施加重心偏移。
inline constexpr double velocity_filter = 0.1; // 速度命令每帧向新目标靠拢的比例；越大响应越快，越小起停越柔和，通常在 0..1 内选。
inline constexpr double maximum_linear_velocity = 1; // MPC 接受的前后/横移合速度上限；超限命令会被拒绝，不会自动缩放到上限。
inline constexpr double maximum_yaw_rate = 2; // MPC 接受的转向速度绝对值上限；超限命令会被拒绝。
inline constexpr double maximum_position_offset = 0.05; // 允许额外添加的水平位置偏移总长度上限；校验位置微调命令，不是常规步长。
inline constexpr double pitch_integral_min_speed = 0.2; // 前向速度超过此门槛才累积俯仰补偿；提高后，低速行走时更少进行俯仰积分。
inline constexpr double roll_integral_min_speed = 0.1; // 横向速度超过此门槛才累积侧倾补偿；提高后，低速横移时更少进行侧倾积分。
inline constexpr double attitude_integral_limit = 0.25; // 侧倾/俯仰补偿积分的限幅；限制长期误差能积累出的补偿量，不是机身安全角度。
inline constexpr double position_error_limit = 0.1; // 积分得到的水平目标位置可领先/落后实测位置的单轴上限；防止目标长期越走越远。
inline constexpr double moving_velocity_squared = 1e-8; // 判定速度命令是否仍在运动的平方门槛；超过后才继续积分水平位置目标。
inline constexpr double moving_yaw_rate = 1e-4; // 判定转向命令是否仍在运动的门槛；与平移门槛共同决定是否更新水平位置积分。
}

namespace wbc
{
// WBC 分两层：运动学误差缩放生成关节位置参考；任务 Kp/Kd 生成加速度要求。
// 关闭偏航位置反馈需关注姿态运动学增益和下方运动姿态 Kp；它们是不同反馈通道。
inline constexpr std::array<double, 3> generic_orientation_kinematic_gain{1, 1, 1}; // 独立 BodyOriTask 的姿态误差到运动学修正的缩放；实际 LocomotionCtrl 使用下面的专用配置。
inline constexpr std::array<double, 3> body_position_kinematic_gain{1, 1, 1}; // 机身位置误差对关节位置参考的修正强度；越大越积极纠正机身位置。
inline constexpr std::array<double, 3> body_orientation_kinematic_gain{1, 1, 0}; // 站立/行走/趴下控制器中姿态误差对关节参考的修正强度；yaw=0 表示此通道不纠正偏航位置误差。
inline constexpr std::array<double, 3> foot_kinematic_gain{1, 1, 1}; // 摆动脚位置误差对关节位置参考的修正强度；越大越积极把脚拉回目标轨迹。
inline constexpr double kinematic_pseudoinverse_threshold = 0.001; // 运动学求逆时忽略过小奇异值的门槛；增大可减弱奇异姿态下的放大，也会放弃更多修正方向。
inline constexpr double dynamic_pseudoinverse_threshold = 0.0001; // 动力学任务求逆时忽略过小奇异值的门槛；影响加速度任务在近奇异姿态下的求解。
// 单独创建通用任务/控制器时的默认值；FSM 在构造时一次性传入各运动配置。
inline constexpr double task_position_kp = 50; // 通用机身位置任务把位置误差转成目标加速度的比例增益；实际运动优先看下方 balance_body_position_kp。
inline constexpr double task_position_kd = 1; // 通用机身位置任务的速度误差阻尼；实际运动优先看下方 balance_body_position_kd。
inline constexpr double task_orientation_kp = 50; // 通用姿态任务把角度误差转成目标角加速度的比例增益；FSM 使用下方各运动的姿态增益。
inline constexpr double task_orientation_kd = 1; // 通用姿态任务的角速度误差阻尼；FSM 使用下方各运动的姿态增益。
inline constexpr double task_foot_kp = 100; // 通用足端任务的位置误差反馈；BalanceStand/Locomotion 构造时使用下方 wbc_foot_position_kp。
inline constexpr double task_foot_kd = 5; // 通用足端任务的速度误差阻尼；BalanceStand/Locomotion 构造时使用下方 wbc_foot_position_kd。
inline constexpr double generic_joint_kp = 100; // 通用 WBC 控制器输出的关节位置 PD 增益；FSM 会在构造时传入对应运动的关节增益。
inline constexpr double generic_joint_kd = 2; // 通用 WBC 控制器输出的关节速度 PD 阻尼；FSM 会在构造时传入对应运动的关节增益。
inline constexpr double generic_floating_base_weight = 3000; // 通用 WBIC 对机身加速度修正的惩罚；越大越不愿偏离任务算出的机身加速度。
inline constexpr double generic_reaction_force_weight = 1; // 通用 WBIC 对接触力修正的惩罚；越大越不愿改变传入的期望接触力。
inline constexpr bool foot_include_floating_base = true; // 足端任务的雅可比是否包含机身自由度；关闭后只用关节运动解释足端运动。
}

// 行走落脚规划、入场过渡及异常回站条件；不是遥控速度档位。
namespace locomotion
{
inline constexpr double maximum_step_correction = 0.30; // 落脚规划中速度/转向修正项的单轴限幅；不是最终整步长度，基础足位和剩余摆动位移另行叠加。
inline constexpr std::array<double, 4> interleave_y{-0.08, 0.08, 0.02, -0.02}; // 各腿在前进时的横向落脚修正系数；与前向速度大小和 interleave_gain 相乘，用于减少腿间干涉。
inline constexpr double interleave_gain = -0.2; // 统一缩放 interleave_y 的作用；绝对值越大横向修正越明显，变号会反转修正方向。
inline constexpr double stance_velocity_gain = 0.5; // 按实测速度和支撑时间预留落脚距离的系数；增大后相同速度下的落脚前馈更远。
inline constexpr double velocity_error_gain = 0.03; // 实测速度与目标速度之差对落脚位置的修正强度；增大后更依靠调整落点来纠正速度偏差。
inline constexpr double yaw_step_gain = 0.5; // 按转向速度和支撑时间预转名义落脚位置的比例；增大后转弯时的落脚旋转修正更强。
inline constexpr double centrifugal_gain = 0.5; // 按机身高度、平移速度和转向速度计算的转弯落脚补偿强度。
inline constexpr double entry_hold_duration = 0.2; // 进入行走后先保持四脚支撑多久，再执行请求的步态；增大后踏步启动更晚。
inline constexpr std::size_t stand_handoff_frames = 2; // 正常退出行走回站立时，融合末帧行走命令与站立命令的帧数；用于减小切换突变。
inline constexpr double entry_height_margin = 0.005; // 允许机身低于名义站高多少仍可进入行走；越大越容易放行尚未完全站高的状态。
inline constexpr double entry_minimum_calf_angle = 0.30; // 允许进入行走的小腿关节角下限；用于拦截仍在收腿/折叠的姿态。
inline constexpr double touchdown_maximum_vertical_speed = 0.20; // 单次踏步落地确认时允许的足端竖直速度绝对值上限；越小越要求脚先稳定下来。
inline constexpr double touchdown_probability = 0.5; // 单次踏步在有效实测接触数据下，认定落地所需的接触概率下限；无有效实测时跳过力确认。
inline constexpr double touchdown_support_ratio = 0.15; // 单次踏步确认落地所需实测支撑力，占四脚均分体重时单脚支撑力的比例。
inline constexpr unsigned int touchdown_confirm_frames = 3; // 单次踏步的落地条件须连续满足多少帧才接受支撑切换；增大可抑制瞬时误判。
inline constexpr unsigned int touchdown_timeout_frames = 50; // 单次踏步等待落地确认的最长帧数；达到后触发安全回站。
inline constexpr double maximum_roll_degrees = 40; // 行走允许的最大左右侧倾；超过后停止继续行走并请求回到站立。
inline constexpr double maximum_pitch_degrees = 40; // 行走允许的最大前后俯仰；超过后停止继续行走并请求回到站立。
inline constexpr double lateral_boundary_tolerance = 0.01; // 足端横向安全边界的额外余量；叠加在 locomotion_max_lateral_foot_offset 上。
inline constexpr double maximum_foot_speed = 9; // 行走时允许的实测足端速度上限；超限视为异常并请求回站，不是摆腿速度目标。
}

// 步态决定何时抬哪条腿；它与“前进/后退/横移”的速度方向命令分别控制。
namespace gait
{
inline constexpr double override_period = 0.5; // override_mode=2 时覆盖预设步态的周期；越小步频越高，普通请求模式不使用它。
inline constexpr double override_stance_ratio = 0.5; // override_mode=2 时覆盖的支撑时间占比；越大脚留在地面的时间越长、摆动时间越短。
inline constexpr unsigned int override_mode = 0; // 步态选择策略：0=跟随请求，1=固定预设，2=预设并覆盖时序，3=跟随自然请求，4=自然时序调节。
inline constexpr unsigned int initial_gait = 0; // 调度器初始步态/预设选择；0=站立，2=TROT；其他编号见 GaitType，之后仍受请求与覆盖策略控制。
inline constexpr unsigned int locomotion_gait = 2; // 行走状态默认请求的步态；2=TROT，3/4=两组互换的单次踏步；不会取消入场四脚保持。
inline constexpr double timing_change_tolerance = 0.0001; // 覆盖周期/支撑比例的变化超过此差值才重算步态；避免微小数值差异反复更新。
inline constexpr double transition_period_scale = 3; // 过渡到站立时相对原周期的拉长倍数；增大后给剩余摆动腿更多落地时间。
struct Profile
{
  double period; // 一个完整抬脚/落脚循环的时长；缩短会提高步频，TROT/单次踏步的周期另由 MPC 时序派生。
  double stance_ratio; // 每条腿在一个周期内支撑的比例；1 表示始终支撑，减小会延长摆动占比。
  std::array<double, 4> phase_offset; // 四腿相对全局周期的相位偏移；相同值表示同步，改变它可重组抬脚顺序。
  double initial_phase; // 创建步态时从周期的哪个位置起步；影响第一组先抬哪条腿及初次切换时间。
};
inline constexpr Profile stand{10, 1, {0.5, 0.5, 0.5, 0.5}, 0}; // 四脚持续支撑的站立预设；支撑比为 1，因此周期和相位不会产生交替抬脚。
inline constexpr Profile amble{0.5, 0.625, {0, 0.5, 0.25, 0.75}, 0}; // 错相慢走预设；每条腿错开抬脚，周期决定节奏，支撑比决定同一时刻的支撑重叠。
inline constexpr double trot_stance_ratio = 0.5; // TROT 和单次踏步共用的支撑比例；增大后缩短每次摆腿时间。
inline constexpr std::array<double, 4> trot_phase_offset{0, 0.5, 0.5, 0}; // TROT 四腿的相位编组；相同值表示同步，默认 FR/RL 与 FL/RR 两组交替。
inline constexpr double single_step_initial_phase = 0.5; // 单次踏步起始相位；决定选中的摆动腿从周期的哪个位置开始执行一次动作。
inline constexpr Profile trot_run{0.4, 0.4, {0, 0.5, 0.5, 0}, 0}; // 跑动小跑预设；默认支撑比小于一半，两组对角腿交替时可出现腾空段。
inline constexpr Profile pace{0.35, 0.5, {0, 0.5, 0, 0.5}, 0.25}; // 同侧步预设；默认左侧两腿同步、右侧两腿同步，两侧交替。
inline constexpr Profile bound{0.4, 0.4, {0, 0, 0.5, 0.5}, 0}; // 前后跳预设；默认前腿一组、后腿一组交替。
inline constexpr Profile rotary_gallop{0.4, 0.2, {0, 0.8571, 0.3571, 0.5}, 0}; // 旋转疾驰命名的预设时序；具体落脚关系由四腿相位表决定。
inline constexpr Profile traverse_gallop{0.5, 0.2, {0, 0.8571, 0.3571, 0.5}, 0}; // 横向疾驰命名的预设时序；当前相位表与 rotary_gallop 相同，周期不同。
inline constexpr Profile pronk{0.5, 0.5, {0, 0, 0, 0}, 0}; // 四脚同步跳预设；四腿相位相同，同步支撑和摆动。
}

// 状态估计：过程噪声越大越不信预测，观测噪声越大越不信对应传感器约束。
namespace estimation
{
inline constexpr double maximum_time_step = 0.1; // 估计器允许连续积分的最大采样间隔；大间隔会被拒绝或重新建立姿态参考，避免跨丢帧积分。
inline constexpr double maximum_sensor_skew = 0.02; // IMU 与关节反馈可被视作同一帧的最大时间差；减小会更严格地拒绝不同步数据。
inline constexpr double sensor_skew_periods = 2; // Runner 同步容差至少覆盖多少个控制周期；实际容差取它乘控制周期与 maximum_sensor_skew 的较大值。
inline constexpr double process_noise_position = 0.02; // 位置预测的不确定度；增大后滤波器更不信任位置预测，更愿接受足端观测修正。
inline constexpr double process_noise_velocity = 0.02; // 速度预测的不确定度；增大后更容易用支撑足观测修正 IMU 积分速度。
inline constexpr double process_noise_foot_position = 0.002; // 足端世界位置预测的不确定度；增大后允许估计的支撑足位置更快调整。
inline constexpr double sensor_noise_relative_position = 0.001; // 关节运动学给出的机身到足端位置观测噪声；增大后更少相信这项位置观测。
inline constexpr double sensor_noise_relative_velocity = 0.1; // 支撑足零速度约束的观测噪声；增大后更少用它约束机身速度。
inline constexpr double sensor_noise_foot_height = 0.001; // 支撑足贴地高度观测的噪声；增大后更少依赖地面高度来校正机身高度。
inline constexpr double initial_covariance = 100; // 滤波器启动/复位时对状态的不确定度；越大初期越容易接受观测修正。
inline constexpr double suspect_noise_multiplier = 100; // 对低可信接触/摆动脚放大过程和观测噪声的强度；越大越少把这些脚当作稳定支撑。
inline constexpr double process_noise_scale = 20; // 位置和速度过程噪声的公共除数；增大后减小这两类预测噪声，与直接增大过程噪声方向相反。
inline constexpr double horizontal_covariance_threshold = 1e-6; // 水平位置协方差行列式超过此门槛时执行去耦和收缩；减小后更容易触发。
inline constexpr double horizontal_covariance_divisor = 10; // 去耦后水平位置协方差的收缩除数；增大后更强地压缩水平不确定度。
inline constexpr double accelerometer_correction_gain = 2; // 用重力方向纠正陀螺积分 roll/pitch 的速度；增大收敛更快，也更易受运动加速度干扰。
inline constexpr double imu_orientation_correction_gain = 10; // 用 IMU 姿态观测纠正积分姿态的强度；增大后更快跟随有效的 IMU 姿态。
inline constexpr double gravity_acceleration_min_ratio = 0.5; // 把加速度当作重力方向的模长下界，相对于当前重力基准；降低后会接受更多低加速度样本。
inline constexpr double gravity_acceleration_max_ratio = 1.5; // 把加速度当作重力方向的模长上界；提高后会接受更多受运动加速度影响的样本。
inline constexpr unsigned int simulation_orientation_mode = 0; // 仿真姿态来源：0=直接使用仿真姿态真值，1=走 IMU 融合路径。
inline constexpr unsigned int hardware_orientation_mode = 1; // 硬件姿态估计模式：1=IMU 融合；0=直接使用输入姿态，跳过融合。
inline constexpr double force_ratio_gain = 8; // 支撑力比例映射为接触概率时的陡峭程度；越大越接近硬阈值，越小越平滑。
inline constexpr double force_ratio_threshold = 0.3; // 接触概率为 0.5 时的实测/参考支撑力比例；提高后同样的力更不容易被视为接触。
inline constexpr double contact_probability_threshold = 0.5; // 接触估计器把概率转成“已接触”的门槛；区别于行走落地确认的独立门槛。
inline constexpr double minimum_support_ratio = 0.2; // 估计接触时参考支撑力的下限，占单脚名义支撑力的比例；防止低载/腾空时力比值异常放大。
inline constexpr double force_estimation_damping = 1e-3; // 由关节力矩反推足端力的最小二乘阻尼；增大可抑制奇异姿态噪声，也会压低力估计。
}

// 普通趴下的贴地、稳定判断与分段收腿；不同于倒地后的 recovery 恢复动作。
namespace prone
{
inline constexpr double ground_press_depth = 0.015; // 趴下完成保持时，将机身目标设为 prone_body_height 再减去此值，帮助机身贴地。
inline constexpr double stable_height_error = 0.015; // 判断下降姿态已稳定时，允许实际高度偏离趴卧目标的范围；越小越严格。
inline constexpr double stable_vertical_speed = 0.05; // 判断下降姿态稳定时，允许的机身竖直速度；越小越要求先停止升降。
inline constexpr double stable_roll_pitch = 0.08; // 判断下降姿态稳定时，允许的侧倾/俯仰范围；越小越要求机身水平。
inline constexpr double stable_angular_speed = 0.15; // 判断下降姿态稳定时，允许的机身总角速度；越小越要求转动充分停止。
inline constexpr double stable_joint_speed = 0.20; // 判断下降姿态稳定时，允许每条腿三个关节速度的合成大小；越小越要求腿先停稳。
inline constexpr std::size_t calf_duration_divisor = 4; // 小腿先行折叠阶段时长 = prone_down_fold_duration / 此值；增大后小腿准备动作更快。
inline constexpr double calf_preparation_progress = 0.5; // 小腿先行阶段完成全部折叠行程的比例；剩余行程留到随后大腿、小腿一起折叠。
inline constexpr double fold_start_height_offset = 0.14; // 实测高度低于 prone_body_height 加此偏移时开始收腿；增大后在更高位置提前收腿。
}

// 恢复站立：收腿 -> 必要时侧滚 -> 展开站起；使用关节 PD，不经过 MPC/WBC。
namespace recovery
{
inline constexpr double rollover_ramp = 0.30; // 恢复站立流程中向侧滚目标姿态插值的时长；增大后侧滚动作更慢。
inline constexpr double rollover_settle = 0.30; // 侧滚动作之后额外等待的时长，给机身翻转和接地留出时间。
inline constexpr double fold_ramp = 0.80; // 恢复流程中向收腿姿态插值的时长；独立于普通趴下的折叠时长。
inline constexpr double fold_settle = 1.40; // 恢复收腿完成后的稳定等待时间；等待后根据是否仍倒置选择侧滚或站起。
inline constexpr double standup_ramp = 0.50; // 恢复流程中从当前关节姿态展开到模型站姿的时长；独立于正常起身 stand_up_duration。
inline constexpr std::array<double, 3> fold_position{0, 1.4, -2.7}; // 恢复动作使用的三关节收腿目标；会先按各腿机械关节限位裁剪。
inline constexpr std::array<double, 3> rolling_right_position{0.8, 1.6, -2.77}; // 侧滚动作时右侧两腿的关节目标；与左侧的不对称姿态共同产生翻转作用。
inline constexpr std::array<double, 3> rolling_left_position{0.8, 3.1, -2.77}; // 侧滚动作时左侧两腿的关节目标；同样受机械限位裁剪。
inline constexpr double standing_height_minimum = 0.20; // 进入恢复状态时，允许直接走站起分支的高度下界；更低时先收腿。
inline constexpr double standing_height_maximum = 0.45; // 进入恢复状态时，允许直接走站起分支的高度上界；还须满足机身未倒置。
inline constexpr double failure_check_progress = 0.7; // 恢复站起完成到此比例后开始检查失败；降低后更早因倒置或高度过低而退回收腿重试。
inline constexpr double low_height = 0.10; // 恢复站起失败判断的最低机身高度；检查开启后低于它会重新收腿。
}

// 软件保护：这些值决定命令裁剪或动作放行条件，不负责生成运动目标。
namespace safety
{
inline constexpr double joint_command_torque_limit = 60; // LegController 最终输出前的每关节总力矩上限；包含 PD 与前馈，超限时缩放该关节命令。
inline constexpr double maximum_roll_pitch = 1.4; // 通用 FSM 姿态安全检查的侧倾/俯仰界限；只有启用该检查的状态使用，行走另有更严格门槛。
inline constexpr double foot_xy_limit_sine = 0.8660254037844386; // 按最大腿长换算足端前后/横向允许范围的比例；越大允许足伸得越远，仅在启用足端检查时使用。
inline constexpr double foot_z_minimum_length_divisor = 4; // 足端安全检查中最短向下伸展长度的除数；增大后允许脚更靠近机身。
inline constexpr double standup_maximum_roll_pitch = 0.20; // 判断是否已经站稳、可以继续动作时允许的侧倾/俯仰；不是 FSM 的极限倾倒角。
inline constexpr double stance_thigh_lower = -0.34; // 额外软件站姿限位中的大腿关节目标下界；仅在站姿限位启用时裁剪 WBC 参考。
inline constexpr double stance_thigh_upper = -0.10; // 额外软件站姿限位中的大腿关节目标上界；与下界共同限制站立支撑构型。
inline constexpr double stance_calf_lower = -0.03; // 额外软件站姿限位中的小腿关节目标下界；区别于机械关节限位。
inline constexpr double stance_calf_upper = 1.20; // 额外软件站姿限位中的小腿关节目标上界；与下界共同约束站立时的小腿角度和支撑腿形。
inline constexpr double stance_limit_maximum_target_height = 0.33; // 只有目标站高不大于此值才启用额外站姿角度限制；更高站姿沿用模型关节范围。
inline constexpr double stance_limit_activation_height = 0.28; // 实测站高达到此值后才锁存启用站姿角度限制，避免起身尚未完成就强行裁剪关节目标。
inline constexpr double maximum_hardware_control_time_step = 0.02; // 硬件 Runner 接受的控制周期上限；用于拒绝过慢的控制周期设置。
}

// 硬件链路的软件默认值：启动检查、反馈时效、调度告警与重连策略。
namespace hardware
{
inline constexpr float torque_limit = 88; // 硬件发送链和仿真执行端默认的每关节力矩上限；与 LegController 限幅同时存在，启动参数可显式覆盖。
inline constexpr double feedback_timeout = 0.10; // MIT 接口判定电机反馈是否过期的时限；增大允许使用更旧的数据，不会增加反馈更新频率。
inline constexpr float startup_zero_tolerance = 0.05F; // 普通启动时关节位置相对标定零位的允许偏差；只检查就绪条件，不会写入电机零位。
inline constexpr float startup_stationary_velocity = 0.05F; // 启动检查允许的关节速度绝对值；减小后更严格地要求电机静止。
inline constexpr int startup_stable_samples = 3; // 普通启动检查要求连续通过的稳定样本数；增大后确认更慢、更不容易被短暂稳定误触发。
inline constexpr int maximum_consecutive_overruns = 3; // 连续控制周期超时达到多少次时触发超时处理/告警；当前处理会丢弃过期周期，不自动失能。
inline constexpr int maximum_overruns_in_window = 5; // 滑动统计窗口内累计超时达到多少次时触发超时处理/告警；与连续次数条件任一满足即可。
inline constexpr double overrun_window = 1; // 统计零散控制超时的时间窗口；相同次数阈值下，增大窗口更容易累积触发。
inline constexpr int startup_sequence_timeout_ms = 100; // 普通启动等待足够稳定样本的时间预算；只读诊断等单轮反馈分支另用自己的等待时限。
inline constexpr unsigned int motor_frame_divider = 4; // 未启用控制输出时，多少个控制周期做一次电机反馈轮询；增大可减轻总线负担，不改变独立 MIT 续发频率。
inline constexpr float startup_maximum_angular_speed = 0.15F; // 启动检查允许的机身总角速度；超过则认为机器人仍在转动。
inline constexpr float startup_gravity_minimum = 7; // 启动时加速度模长的有效下界；用于排除明显非静止或异常 IMU 样本。
inline constexpr float startup_gravity_maximum = 12; // 启动时加速度模长的有效上界；与下界共同判断样本是否接近正常重力。
inline constexpr float startup_maximum_tilt = 0.25F; // 启动检查允许的机身倾斜；根据横向重力分量判断，增大后允许更倾斜地启动。
inline constexpr double imu_loss_before_recovery = 0.5; // IMU 连续读取失败达到多久才尝试重新连接；增大后更容忍短时抖动。
inline constexpr double imu_recovery_interval = 2; // 两次 IMU 重连尝试之间的最小间隔；增大后重试更慢，减少反复打开串口。
}

// MPC/WBIC 共用的 QP 求解选项，影响数值稳定性和求解工作量。
namespace solver
{
inline constexpr int maximum_working_set_recalculations = 100; // QP 求解器每次求解最多进行的工作集更新次数；增大可能提高困难问题的求解机会，也可能增加耗时。
inline constexpr double infinity = 1e20; // 交给 QP 求解器表示“无实际边界”的大数；不是物理力或位置上限。
inline constexpr bool regularisation = true; // 是否启用 QP 正则化以改善近退化问题的数值条件；可能对原优化问题施加很小的修正。
}

// 类型转换和时序派生，只传递配置数值，不在调用处写第二份默认值。
// 将配置数组转换为控制器所用的浮点类型，不改变数组内容或顺序。
template<typename T, std::size_t N>
std::array<T, N> array(const std::array<double, N> & values)
{
  std::array<T, N> result{};
  for (std::size_t i = 0; i < N; ++i) {result[i] = static_cast<T>(values[i]);}
  return result;
}
// 将三分量配置转换为控制算法使用的向量。
template<typename T>
Vec3<T> vector3(const std::array<double, 3> & values)
{return Vec3<T>(T(values[0]), T(values[1]), T(values[2]));}
// 将四腿相位等四分量配置转换为向量，保持 FR/FL/RR/RL 顺序。
template<typename T>
Vec4<T> vector4(const std::array<double, 4> & values)
{return Vec4<T>(T(values[0]), T(values[1]), T(values[2]), T(values[3]));}
// 将一个预测段换算成最接近的整数控制帧，至少一帧，保证求解与控制循环对齐。
template<typename T>
std::size_t segmentIterations(T period, T segment = T(mpc::time_step))
{
  const T frames = std::round(segment / period);
  return frames < T(1) ? std::size_t(1) : static_cast<std::size_t>(frames);
}
} // namespace control_config

/** 每次运行创建一份，各状态借用同一份只读配置。这里是各运动实际使用的增益。
 * 机身/足端任务 Kp/Kd 先影响 WBC 的加速度目标；joint_kp/kd 才是直接发送给电机的 PD。
 * 下面按初始化、平衡支撑、正常起身、行走分组；共用参数已在各字段注释中注明。
 */
template<typename T>
struct RobotControlParameters
{
  T minimum_standing_height = T(control_config::minimum_standing_height); // 承接公共站高下限供状态和 Runner 使用；调整数值应改上方公共配置。
  T maximum_standing_height = T(control_config::maximum_standing_height); // 承接公共站高上限供状态和 Runner 使用；调整数值应改上方公共配置。
  T standing_height_rate = T(0.04); // Runner 将站高目标逐渐推向用户设定值的速度；增大后站高升降更快，还受 balance_height_step 限制。
  T prone_body_height = T(0.12); // 趴下时机身下降的基础目标高度；收腿触发高度和贴地保持目标也由它派生。
  T prone_down_height_rate = T(0.08); // 正常趴下时机身目标高度降低的速度；增大后机身更快下沉，不改变关节折叠时长。
  T prone_down_fold_duration = T(2.0); // 普通趴下中大腿与小腿联合折叠阶段的时长；前置小腿准备阶段另取其一部分。
  T balance_height_step = T(0.00016); // BalanceStand 每帧允许的最大高度目标变化；对 Runner 的站高限速再做一层限制，二者较严格者生效。
  T joint_initialization_duration = T(1.2); // 电机使能后，从实测关节位置平滑移动到初始化目标的时长；增大后初始化更慢。
  // 初始化与趴卧保持共用同一 PD。
  Vec3<T> prone_home_joint_kp = Vec3<T>::Constant(T(70)); // 初始化和趴卧保持共用的电机关节位置刚度；增大后更用力保持目标关节角。
  Vec3<T> prone_home_joint_kd = Vec3<T>::Constant(T(1.6)); // 初始化和趴卧保持共用的电机关节速度阻尼；增大后更强地抑制关节运动和振荡。
  bool start_in_prone_home = true; // 是否按机械趴卧零位启动并保持；开启时要先完成显式起身，才允许普通站立/行走。
  Vec3<T> motor_zero_position = Vec3<T>::Zero(); // 趴卧启动/保持时使用的电机坐标关节目标；这是控制目标，不是改写硬件零点或标定。

  Vec3<T> balance_body_position_kp = Vec3<T>::Constant(T(100)); // 站立、行走和趴下共用的机身位置 WBC 比例增益；越大越积极纠正位置/高度误差。
  Vec3<T> balance_body_position_kd = Vec3<T>::Constant(T(10)); // 站立、行走和趴下共用的机身速度误差阻尼；越大越强地抑制机身平移和升降振荡。
  Vec3<T> balance_body_orientation_kp = Vec3<T>(T(200), T(200), T(0)); // 平衡站立的姿态误差反馈；增大对应轴后更积极扶正，默认 yaw=0 不追踪偏航角误差。
  Vec3<T> balance_body_orientation_kd = Vec3<T>(T(10), T(10), T(0)); // 平衡站立的角速度误差阻尼；默认 yaw=0 不通过该动态反馈通道纠正偏航速度。
  Vec3<T> balance_joint_kp = Vec3<T>(T(8), T(8), T(15)); // 平衡站立最终下发到电机的关节位置 PD 增益；决定关节参考跟踪的刚度，不是机身任务增益。
  Vec3<T> balance_joint_kd = Vec3<T>(T(1.8), T(1.8), T(2)); // 平衡站立最终下发到电机的关节速度阻尼；增大后更强地抑制关节速度偏差。
  Vec3<T> joint_kp = Vec3<T>::Constant(T(10)); // 行走及趴下折叠共用的电机关节位置 PD 增益；增大后关节更硬地追踪规划角度。
  Vec3<T> joint_kd = Vec3<T>::Constant(T(2)); // 行走及趴下折叠共用的电机关节速度 PD 阻尼；不是足端任务的速度增益。
  T balance_floating_base_weight = T(100); // 站立和趴下 WBIC 对机身加速度修正的惩罚；越大越优先保留机身任务的加速度要求。
  T balance_reaction_force_weight = T(1); // 站立和趴下 WBIC 对接触力修正的惩罚；越大越倾向保留传入的期望支撑力分配。
  T maximum_normal_force = T(control_config::maximum_normal_force); // 承接公共单脚支撑力上限，传给各状态的 MPC/WBC；统一调节应改上方同名公共配置。

  // 正常起身：从趴卧展开与非趴卧足端起身走不同分支；恢复动作只共用关节 PD。
  T stand_up_duration = T(3.2); // 正常趴卧起身时髋/小腿展开时长的基准；增大后起身更慢，大腿时长还要除以下面的比例。
  T stand_up_thigh_speed_scale = T(0.75); // 起身大腿的相对速度比例；大腿时长 = stand_up_duration / 此值，越小大腿展开越慢。
  Vec3<T> stand_up_prepare_position = Vec3<T>(T(0), T(0.1), T(-2.15)); // 正常趴卧起身展开阶段的三关节目标姿态；改变它会改变起身过程中的腿形。
  Vec3<T> stand_up_joint_kp = Vec3<T>(T(80), T(80), T(90)); // 正常起身关节展开和恢复站立动作共用的电机关节位置刚度。
  Vec3<T> stand_up_joint_kd = Vec3<T>::Constant(T(1.8)); // 正常起身关节展开和恢复站立动作共用的电机关节速度阻尼。
  Vec3<T> stand_up_cartesian_kp = Vec3<T>(T(190), T(190), T(260)); // 非趴卧起身路径的足端位置反馈增益；用于把脚推向起身目标，不作用于趴卧关节展开分支。
  Vec3<T> stand_up_cartesian_kd = Vec3<T>(T(10), T(10), T(14)); // 非趴卧起身路径的足端速度阻尼；与 stand_up_cartesian_kp 配合抑制足端偏差。

  // 行走：MPC 管力和机身参考，WBC 管任务跟踪，电机 PD 管最终关节参考跟踪。
  Vec3<T> locomotion_body_orientation_kp = Vec3<T>(T(200), T(200), T(0)); // 行走和趴下共用的机身姿态反馈；roll/pitch 用于扶正，默认 yaw=0 不追踪偏航角误差。
  Vec3<T> locomotion_body_orientation_kd = Vec3<T>(T(10), T(10), T(0)); // 行走和趴下共用的机身角速度误差阻尼；增大对应轴后更强地抑制旋转偏差。
  T locomotion_swing_height = T(0.04); // 摆动脚相对起点的抬脚高度；增大更利于越障，但同样摆动时间内要求更大的脚速和加速度。
  T locomotion_max_lateral_foot_offset = T(0.30); // 行走时足端横向位置的基础安全边界；超出它加容差后回站，不是横向速度或步宽目标。
  T mpc_time_step = T(control_config::mpc::time_step); // 承接上方 MPC 段长，向 FSM 传递时序；统一修改预测段长应改 control_config::mpc::time_step。
  Vec3<T> wbc_foot_position_kp = Vec3<T>::Constant(T(500)); // 站立/行走创建足端任务时使用的位置反馈增益，主要在摆腿任务激活时影响轨迹跟踪。
  Vec3<T> wbc_foot_position_kd = Vec3<T>::Constant(T(10)); // 站立/行走足端任务的速度误差阻尼；配合位置反馈抑制摆动脚振荡。
  T wbc_foot_velocity_damping = T(7); // 行走 WBC 输出外额外叠加的笛卡尔足速阻尼；经雅可比转为关节力矩，不等同于上面的任务 Kd。
  T locomotion_floating_base_weight = T(100); // 行走 WBIC 对机身加速度修正的惩罚；越大越倾向保住机身任务，允许更多地调整接触力。
  T locomotion_reaction_force_weight = T(1); // 行走 WBIC 对 MPC 接触力修正的惩罚；越大越倾向保留 MPC 的力分配，允许更多机身加速度修正。

};
#endif // MYMIT_ROBOT_CONFIG_CONTROL_CONFIG_H_
