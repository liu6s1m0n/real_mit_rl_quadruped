//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation_ros2/StateEstimateBase.hpp"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>

namespace legged {
using namespace ocs2;

class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics, const rclcpp::Node::SharedPtr& nh);

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);

 protected:
  void updateFromTopic();

  void callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  nav_msgs::msg::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

 private:
  size_t numContacts_, dimContacts_, numState_, numObserve_;

  matrix_t a_, b_, c_, q_, p_, r_;
  vector_t xHat_, ps_, vs_;

  // Topic
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
};

}  // namespace legged
