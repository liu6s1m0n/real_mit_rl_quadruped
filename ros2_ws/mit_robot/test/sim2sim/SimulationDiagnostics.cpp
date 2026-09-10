#include "SimulationDiagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "RobotRunner.hpp"

namespace
{
constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};

std::array<float, 3> meanVector(
  const double (&sum)[3], std::size_t samples) noexcept
{
  std::array<float, 3> result{};
  if (samples == 0) {return result;}
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = static_cast<float>(sum[index] / static_cast<double>(samples));
  }
  return result;
}

float vectorNorm3(const mjtNum (&value)[6]) noexcept
{
  return static_cast<float>(std::sqrt(
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]));
}
}

std::size_t countSimulationTargetJointLimitHits(const RobotRunner & runner)
{
  constexpr float kLimitTolerance = 1.0e-4F;
  std::size_t count = 0;
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & joints = runner.quadruped().leg(static_cast<LegId>(leg)).joints;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto index = static_cast<Eigen::Index>(joint);
      const float target = commands[leg].position_desired(index);
      if (std::abs(target - static_cast<float>(joints.lower_limit(index))) <=
        kLimitTolerance ||
        std::abs(target - static_cast<float>(joints.upper_limit(index))) <=
        kLimitTolerance)
      {
        ++count;
      }
    }
  }
  return count;
}

std::array<float, 3> SimulationDirectionWindow::meanCommand() const noexcept
{
  return meanVector(command_sum, samples);
}

std::array<float, 3> SimulationDirectionWindow::meanActual() const noexcept
{
  return meanVector(actual_sum, samples);
}

float SimulationFootContactStats::meanContactForce() const noexcept
{
  return contact_samples == 0 ? 0.0F : static_cast<float>(
    contact_force_sum / static_cast<double>(contact_samples));
}

float SimulationFootContactStats::meanSlipSpeed() const noexcept
{
  return contact_samples == 0 ? 0.0F : static_cast<float>(
    slip_speed_sum / static_cast<double>(contact_samples));
}

float SimulationDiagnosticReport::rmsPositionError() const noexcept
{
  return observed_frames == 0 ? 0.0F : static_cast<float>(
    std::sqrt(squared_position_error_sum / static_cast<double>(observed_frames)));
}

float SimulationDiagnosticReport::rmsVelocityError() const noexcept
{
  return observed_frames == 0 ? 0.0F : static_cast<float>(
    std::sqrt(squared_velocity_error_sum / static_cast<double>(observed_frames)));
}

float SimulationDiagnosticReport::calfContactRatio() const noexcept
{
  return observed_frames == 0 ? 0.0F : static_cast<float>(
    static_cast<double>(calf_collision_frames) /
    static_cast<double>(observed_frames));
}

float SimulationDiagnosticReport::meanFootSlipSpeed() const noexcept
{
  return foot_contact_samples == 0 ? 0.0F : static_cast<float>(
    foot_slip_speed_sum / static_cast<double>(foot_contact_samples));
}

float SimulationDiagnosticReport::netHorizontalDisplacement() const noexcept
{
  return std::sqrt(
    net_displacement_body_x * net_displacement_body_x +
    net_displacement_body_y * net_displacement_body_y);
}

int SimulationDiagnostics::requireObject(
  const mjModel * model, int type, const char * name)
{
  const int id = mj_name2id(model, type, name);
  if (id < 0) {throw std::runtime_error(std::string("MuJoCo object not found: ") + name);}
  return id;
}

SimulationDiagnostics::SimulationDiagnostics(const mjModel * model)
: model_(model)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("SimulationDiagnostics requires a MuJoCo model");
  }
  trunk_body_ = requireObject(model_, mjOBJ_BODY, "trunk");
  floor_geom_ = requireObject(model_, mjOBJ_GEOM, "floor");
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_geoms_[leg] = requireObject(model_, mjOBJ_GEOM, kLegNames[leg]);
    const std::string foot_name = std::string(kLegNames[leg]) + "_foot";
    foot_bodies_[leg] = requireObject(model_, mjOBJ_BODY, foot_name.c_str());
    const std::string calf_name = std::string(kLegNames[leg]) + "_calf";
    calf_bodies_[leg] = requireObject(model_, mjOBJ_BODY, calf_name.c_str());
  }
  reset();
}

void SimulationDiagnostics::reset() noexcept
{
  position_offset_.setZero();
  position_offset_initialized_ = false;
  active_direction_time_s_ = 0.0;
  active_start_position_world_.setZero();
  active_start_yaw_ = 0.0F;
  active_start_initialized_ = false;
  previous_foot_contact_.fill(false);
  report_ = SimulationDiagnosticReport{};
}

Vec3<float> SimulationDiagnostics::bodyPosition(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  return Eigen::Map<const Vec3<double>>(data->xpos + 3 * trunk_body_).cast<float>();
}

Vec3<float> SimulationDiagnostics::bodyLinearVelocity(const mjData * data) const
{
  const Vec3<float> world_velocity = bodyLinearVelocityWorld(data);
  const mjtNum * rotation = data->xmat + 9 * trunk_body_;
  const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> body_to_world(
    rotation);
  return body_to_world.transpose().cast<float>() * world_velocity;
}

Vec3<float> SimulationDiagnostics::bodyLinearVelocityWorld(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  mjtNum velocity[6]{};
  mj_objectVelocity(model_, data, mjOBJ_BODY, trunk_body_, velocity, 0);
  return Eigen::Map<const Vec3<double>>(velocity + 3).cast<float>();
}

Vec3<float> SimulationDiagnostics::bodyAngularVelocity(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  mjtNum velocity[6]{};
  mj_objectVelocity(model_, data, mjOBJ_BODY, trunk_body_, velocity, 0);
  const mjtNum * rotation = data->xmat + 9 * trunk_body_;
  const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> body_to_world(
    rotation);
  return body_to_world.transpose().cast<float>() *
    Eigen::Map<const Vec3<double>>(velocity).cast<float>();
}

float SimulationDiagnostics::bodyYaw(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  const mjtNum * rotation = data->xmat + 9 * trunk_body_;
  return static_cast<float>(std::atan2(rotation[3], rotation[0]));
}

float SimulationDiagnostics::orientationError(
  const mjData * data, const Eigen::Quaternionf & estimate) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  const mjtNum * value = data->xquat + 4 * trunk_body_;
  const Eigen::Quaternionf truth(
    static_cast<float>(value[0]), static_cast<float>(value[1]),
    static_cast<float>(value[2]), static_cast<float>(value[3]));
  return Eigen::AngleAxisf(truth.conjugate() * estimate).angle();
}

bool SimulationDiagnostics::hasCalfCollision(const mjData * data) const
{
  if (data == nullptr) {throw std::invalid_argument("diagnostics data is null");}
  for (int index = 0; index < data->ncon; ++index) {
    const mjContact & contact = data->contact[index];
    const int other = contact.geom1 == floor_geom_ ? contact.geom2 :
      (contact.geom2 == floor_geom_ ? contact.geom1 : -1);
    if (other < 0 ||
      std::find(foot_geoms_.begin(), foot_geoms_.end(), other) != foot_geoms_.end())
    {
      continue;
    }
    const int body = model_->geom_bodyid[other];
    if (std::find(calf_bodies_.begin(), calf_bodies_.end(), body) !=
      calf_bodies_.end())
    {
      return true;
    }
  }
  return false;
}

void SimulationDiagnostics::observe(
  const mjData * data, const StateEstimate<float> & estimate,
  bool control_valid, bool direction_active, const Vec3<float> & command_body,
  std::size_t target_joint_limit_hits)
{
  if (!estimate.valid) {
    ++report_.rejected_control_frames;
    return;
  }
  const Vec3<float> truth_position = bodyPosition(data);
  if (!position_offset_initialized_) {
    // 足端里程计无法观测绝对水平原点；与测试相同，仅校准一次x/y常量偏移。
    position_offset_ = truth_position - estimate.position_world;
    position_offset_.z() = 0.0F;
    position_offset_initialized_ = true;
  }
  const float position_error =
    (estimate.position_world + position_offset_ - truth_position).norm();
  const float velocity_error =
    (estimate.velocity_world - bodyLinearVelocityWorld(data)).norm();
  const Vec3<float> actual_body_velocity = bodyLinearVelocity(data);
  const Vec3<float> actual_body_angular_velocity = bodyAngularVelocity(data);
  report_.command_vx = command_body.x();
  report_.command_vy = command_body.y();
  report_.command_wz = command_body.z();
  report_.actual_vx = actual_body_velocity.x();
  report_.actual_vy = actual_body_velocity.y();
  report_.actual_wz = actual_body_angular_velocity.z();
  if (direction_active) {
    if (!active_start_initialized_) {
      active_start_position_world_ = truth_position;
      active_start_yaw_ = bodyYaw(data);
      active_start_initialized_ = true;
    }
    const Vec3<float> delta_world = truth_position - active_start_position_world_;
    const float delta_yaw = active_start_yaw_;
    report_.net_displacement_body_x =
      std::cos(delta_yaw) * delta_world.x() + std::sin(delta_yaw) * delta_world.y();
    report_.net_displacement_body_y =
      -std::sin(delta_yaw) * delta_world.x() + std::cos(delta_yaw) * delta_world.y();
  }
  report_.maximum_position_error =
    std::max(report_.maximum_position_error, position_error);
  report_.maximum_velocity_error =
    std::max(report_.maximum_velocity_error, velocity_error);
  report_.maximum_orientation_error = std::max(
    report_.maximum_orientation_error,
    orientationError(data, estimate.orientation_world_from_body));
  report_.maximum_absolute_pitch =
    std::max(report_.maximum_absolute_pitch, std::abs(estimate.rpy.y()));
  report_.minimum_height = std::min(report_.minimum_height, estimate.position_world.z());
  report_.squared_position_error_sum += position_error * position_error;
  report_.squared_velocity_error_sum += velocity_error * velocity_error;
  report_.calf_collision_frames += hasCalfCollision(data) ? 1U : 0U;
  if (direction_active) {
    active_direction_time_s_ += static_cast<double>(model_->opt.timestep);
  }
  const double elapsed = active_direction_time_s_;
  const std::size_t window = elapsed < 1.0 ? 0 :
    (elapsed < 2.0 ? 1 : (elapsed < 5.0 ? 2 : 3));
  if (direction_active && elapsed <= 10.0) {
    auto & direction = report_.direction_windows[window];
    direction.command_sum[0] += command_body.x();
    direction.command_sum[1] += command_body.y();
    direction.command_sum[2] += command_body.z();
    direction.actual_sum[0] += actual_body_velocity.x();
    direction.actual_sum[1] += actual_body_velocity.y();
    direction.actual_sum[2] += actual_body_angular_velocity.z();
    ++direction.samples;
  }
  report_.target_joint_limit_hits += target_joint_limit_hits;

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    bool foot_contact = false;
    float contact_force = 0.0F;
    float slip_speed = 0.0F;
    for (int index = 0; index < data->ncon; ++index) {
      const mjContact & contact = data->contact[index];
      if ((contact.geom1 == floor_geom_ && contact.geom2 == foot_geoms_[leg]) ||
        (contact.geom2 == floor_geom_ && contact.geom1 == foot_geoms_[leg]))
      {
        foot_contact = true;
        mjtNum force[6]{};
        mj_contactForce(model_, data, index, force);
        contact_force += vectorNorm3(force);
      }
    }
    auto & contact_stats = report_.foot_contact_by_leg[leg];
    if (foot_contact != previous_foot_contact_[leg]) {
      ++contact_stats.contact_transitions;
    }
    previous_foot_contact_[leg] = foot_contact;
    if (foot_contact) {
      mjtNum velocity[6]{};
      mj_objectVelocity(model_, data, mjOBJ_BODY, foot_bodies_[leg], velocity, 0);
      slip_speed = std::sqrt(
        static_cast<float>(velocity[3] * velocity[3] + velocity[4] * velocity[4]));
      report_.maximum_foot_slip_speed =
        std::max(report_.maximum_foot_slip_speed, slip_speed);
      report_.foot_slip_speed_sum += slip_speed;
      ++report_.foot_contact_samples;
      ++contact_stats.contact_samples;
      contact_stats.contact_force_sum += contact_force;
      contact_stats.maximum_contact_force = std::max(
        contact_stats.maximum_contact_force, contact_force);
      contact_stats.slip_speed_sum += slip_speed;
      contact_stats.maximum_slip_speed = std::max(
        contact_stats.maximum_slip_speed, slip_speed);
    }
  }
  const mjtNum * rotation = data->xmat + 9 * trunk_body_;
  const float roll = static_cast<float>(std::atan2(rotation[7], rotation[8]));
  const float pitch = static_cast<float>(std::atan2(
    -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3])));
  if (truth_position.z() < 0.20F || std::abs(roll) > 0.8F || std::abs(pitch) > 0.8F) {
    report_.fall_time_s += static_cast<float>(model_->opt.timestep);
  }
  report_.rejected_control_frames += control_valid ? 0U : 1U;
  ++report_.observed_frames;
}
