/** @file robot_control_parameters.hpp
 * @brief 控制配置的兼容入口；所有调参数值统一在 config/control_config.h。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
#define MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_

#include <stdexcept>
#include "config/control_config.h"

template<typename T>
RobotControlParameters<T> makeRobotControlParameters(RobotType robot_type)
{
  if (robot_type != RobotType::DM1) {
    throw std::invalid_argument("only the DM1 control profile is supported");
  }
  return RobotControlParameters<T>{};
}

template<typename T>
const RobotControlParameters<T> & defaultControlParameters()
{
  static const RobotControlParameters<T> parameters{};
  return parameters;
}

template<typename T>
std::size_t mpcSegmentIterations(
  const RobotControlParameters<T> & parameters, T control_time_step)
{
  if (!(control_time_step > T(0)) || !(parameters.mpc_time_step > T(0))) {
    throw std::invalid_argument("MPC timing requires positive time steps");
  }
  return control_config::segmentIterations(control_time_step, parameters.mpc_time_step);
}
#endif // MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
