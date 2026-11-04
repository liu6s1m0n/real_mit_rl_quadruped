//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation_ros2/StateEstimateBase.hpp"

#include <realtime_tools/realtime_buffer.hpp>

#pragma once
namespace legged {
using namespace ocs2;

class FromTopicStateEstimate : public StateEstimateBase {
 public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics, const rclcpp::Node::SharedPtr& nh);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal, const vector3_t& linearAccelLocal,
                 const matrix3_t& orientationCovariance, const matrix3_t& angularVelCovariance,
                 const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
};

}  // namespace legged
