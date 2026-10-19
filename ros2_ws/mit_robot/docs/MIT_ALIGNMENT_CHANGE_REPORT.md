# MIT 对齐修改记录

## MPC 求解诊断

作用：在不改变 MPC 状态方程、代价函数、约束或输出的前提下，增加可选的
QP 耗时、求解失败和最小约束裕量日志。设置 `DM1_MPC_DIAGNOSTICS=1` 时启用。

修改前：

```cpp
VecD solution = VecD::Zero(n);
const bool solved = qp_adapter::solve(
  hessian, gradient, qp_constraints, variable_lower, variable_upper,
  constraint_lower, constraint_upper, solution, &cost);
if (!std::isfinite(cost) || !solution.allFinite()) {return SolverResult<T>{};}
```

修改后：

```cpp
VecD solution = VecD::Zero(n);
const bool diagnostics = std::getenv("DM1_MPC_DIAGNOSTICS") != nullptr &&
  std::getenv("DM1_MPC_DIAGNOSTICS")[0] == '1';
const auto qp_start = std::chrono::steady_clock::now();
const bool solved = qp_adapter::solve(
  hessian, gradient, qp_constraints, variable_lower, variable_upper,
  constraint_lower, constraint_upper, solution, &cost);
const double qp_elapsed_ms = std::chrono::duration<double, std::milli>(
  std::chrono::steady_clock::now() - qp_start).count();
if (!std::isfinite(cost) || !solution.allFinite()) {
  if (diagnostics) {
    std::fprintf(stderr, "[MPC][DIAGNOSTIC] solve_failed qp_ms=%.6f\n", qp_elapsed_ms);
  }
  return SolverResult<T>{};
}
if (diagnostics) {
  double minimum_constraint_margin = std::numeric_limits<double>::infinity();
  for (Eigen::Index constraint = 0; constraint < constraints; ++constraint) {
    minimum_constraint_margin = std::min(
      minimum_constraint_margin,
      ci.col(constraint).dot(solution) + ci0(constraint));
  }
  std::fprintf(stderr,
    "[MPC][DIAGNOSTIC] horizon=%zu contact_variables=%zu cost=%.9g "
    "min_constraint_margin=%.9g qp_ms=%.6f\n",
    horizon, variables.size(), cost, minimum_constraint_margin, qp_elapsed_ms);
}
```

## MPC 接触力约束回归测试

作用：验证首段支撑力满足 MIT Convex MPC 使用的法向力上下限和四棱锥摩擦约束，
摆动腿首段力为零。该修改只增加测试，不改变运行时控制逻辑。

修改前：

```cpp
// 原有测试只检查接触状态和步态时序，没有检查 MPC 首段反力约束。
```

修改后：

```cpp
TEST(MpcLocomotion, KeepsFirstStepForcesInsideMitContactConstraints)
{
  const auto quadruped = robots::dm1::makeModel<double>();
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 13);
  controller.setGait(GaitType::TROT);
  const auto result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  ASSERT_TRUE(result.valid);
  const auto & settings = controller.solverSettings();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & force = result.reaction_forces_world[leg];
    ASSERT_TRUE(force.allFinite());
    if (!result.contact_state[leg]) {
      EXPECT_TRUE(force.isApprox(Vec3<double>::Zero(), 1.0e-10));
      continue;
    }
    EXPECT_GE(force.z(), settings.minimum_normal_force - 1.0e-10);
    EXPECT_LE(force.z(), settings.maximum_normal_force + 1.0e-10);
    EXPECT_LE(std::abs(force.x()),
      settings.friction_coefficient * force.z() + 1.0e-10);
    EXPECT_LE(std::abs(force.y()),
      settings.friction_coefficient * force.z() + 1.0e-10);
  }
}
```

## WBC 数据连贯性和动力学残差诊断

作用：对照 MIT WBC 的连续控制数据，记录相邻周期的时间戳、关节状态、WBC
目标、前馈力矩、接触切换、动力学残差和接触约束裕量。诊断只输出日志，不改变
任务、求解器、命令、站高或电机使能状态。

修改前：

```cpp
const auto timing_start = std::chrono::steady_clock::now();
const bool success = run(input, estimate, joint_states);
applyResult(leg_controller);
const double elapsed_ms = ...;
const char * diagnostics = std::getenv("DM1_CONTROL_DIAGNOSTICS");
```

修改后：

```cpp
const char * diagnostics = std::getenv("DM1_CONTROL_DIAGNOSTICS");
const bool trace = diagnostics && diagnostics[0] == '1';
const auto previous = trace ? std::make_unique<Result>(result_) : nullptr;
const T previous_timestamp = estimate_.timestamp;
const DVec<T> previous_q = trace ? state_.q : DVec<T>{};
const DVec<T> previous_qd = trace ? state_.qd : DVec<T>{};
const auto previous_contacts = trace ? contacts_ : std::vector<ContactSpec<T> *>{};
const auto timing_start = std::chrono::steady_clock::now();
const bool success = run(input, estimate, joint_states);
applyResult(leg_controller);

if (trace) {
  // 输出 dt、状态变化、目标变化、接触变化、动力学残差和约束裕量。
  std::fputs(line.str().c_str(), stderr);
}
```

## 连续性回归测试

作用：注入重复时间戳、时间回退、非有限关节数据和接触切换，确认诊断能识别
不可比较的周期，并确认开启诊断前后控制输出一致。测试不添加高度保护和主动失能逻辑。

修改前：

```cpp
// 原有契约测试没有覆盖 WBC 跨周期数据连续性。
```

修改后：

```cpp
TEST(Dm1Contract, WbcTraceObservesRepeatedTimeFailuresAndRecoveryWithoutChangingCommands)
{
  ::testing::internal::CaptureStderr();
  { DiagnosticEnvironment diagnostics("1");
    logged_success = logged.runAndApply(&input, estimate, joint_states, logged_legs); }
  const auto log = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(logged_success, quiet_success);
  ASSERT_NE(log.find("[WBC][CONTINUITY]"), std::string::npos);
  EXPECT_NE(log.find("comparable=0"), std::string::npos);
  EXPECT_NE(log.find("contact_changed=1"), std::string::npos);
  EXPECT_NE(log.find("dynamics_residual="), std::string::npos);
  // 诊断开启前后的 q_des、dq_des、tau_ff 逐关节相同。
}

TEST(Dm1Contract, MpcTracePreservesForceTrajectoryAndReportsTimeStep)
{
  { DiagnosticEnvironment diagnostics("0", "DM1_MPC_DIAGNOSTICS");
    quiet = solver.solve(state, trajectory, contacts); }
  ::testing::internal::CaptureStderr();
  { DiagnosticEnvironment diagnostics("1", "DM1_MPC_DIAGNOSTICS");
    logged = solver.solve(state, trajectory, contacts); }
  const auto log = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(quiet.force_trajectory, logged.force_trajectory);
  EXPECT_EQ(quiet.cost, logged.cost);
  EXPECT_NE(log.find("t_s=2.5 dt_mpc_s=0.030"), std::string::npos);
  EXPECT_NE(log.find("min_constraint_margin="), std::string::npos);
}
```
