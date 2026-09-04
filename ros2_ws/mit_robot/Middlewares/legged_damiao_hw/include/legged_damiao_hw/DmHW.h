
#pragma once

#include <legged_hw/LeggedHW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <controller_interface/controller.h>
#include <unordered_map>
#include <vector>
#include <dmbot_serial/protocol/damiao.h>

// #include <Eigen/Core>
// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/multibody/data.hpp>



namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct DmMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
};
struct DmImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

 class DmHW : public LeggedHW
 {
 public:
  DmHW() {
  }
  ~DmHW() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  //void updateJoystick(const ros::Time& time);
  //void updateContact(const ros::Time& time);

 private:
  bool setupJoints();
  bool setupImu();
  bool setupContactSensor(ros::NodeHandle& nh);


  std::shared_ptr<damiao::Motor_Control> motorsInterface;
  std::shared_ptr<damiao::Motor_Control> motorsInterface2;

  DmMotorData jointData_[12]{};
  DmMotorData dmSendcmd_[12];
  std::vector<damiao::DmActData> CAN1;
  std::vector<damiao::DmActData> CAN2;
  bool contactState_[4]{};  //

  int powerLimit_{};
  int contactThreshold_{};

  const std::vector<int> directionMotor_LeftF{1, 1, 1};
  const std::vector<int> directionMotor_LeftH{-1, 1, 1};
  const std::vector<int> directionMotor_RightF{1, -1, -1};//456
  const std::vector<int> directionMotor_RightH{-1, -1, -1};
  const std::vector<int> send_RH{-1, -1, 1};

  ros::Publisher joyPublisher_;
  ros::Publisher contactPublisher_;
  ros::Time lastJoyPub_, lastContactPub_;

  struct JointMapping {
    std::string name;
    std::size_t qIndex{0};
    std::size_t vIndex{0};
  };

  DmImuData imuData_{};
  ros::Subscriber odom_sub_;
  sensor_msgs::Imu yesenceIMU_;
  std::unordered_map<std::string, DmMotorData*> jointNameToMotorData_;
  void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom) {
    yesenceIMU_ = *odom;
  }

};

}  // namespace legged