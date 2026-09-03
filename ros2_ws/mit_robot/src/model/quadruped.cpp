#include "model/quadruped.hpp"

#include <stdexcept>

#include "model/robots/dm1.hpp"

template<typename T>
Quadruped<T> makeQuadruped(RobotType robot_type)
{
  if (robot_type == RobotType::DM1) {return robots::dm1::makeModel<T>();}
  throw std::invalid_argument("only the DM1 model is supported");
}

// 通用容器和统一工厂当前只预编译 float 精度。
template class Quadruped<float>;
template Quadruped<float> makeQuadruped<float>(RobotType);
