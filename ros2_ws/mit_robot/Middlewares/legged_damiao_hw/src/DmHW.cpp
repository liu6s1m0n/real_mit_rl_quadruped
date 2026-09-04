#include "legged_damiao_hw/DmHW.h"

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include "std_msgs/Float64MultiArray.h"
#include <dmbot_serial/protocol/damiao.h>

// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/multibody/model.hpp>
// #include <pinocchio/multibody/data.hpp>
// #include <pinocchio/algorithm/jacobian.hpp>
// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/algorithm/rnea.hpp>
// #include <Eigen/Dense>

namespace legged 
{
  bool DmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) 
  {
    odom_sub_ = root_nh.subscribe("/imu/data", 1, &DmHW::OdomCallBack, this);
    if (!LeggedHW::init(root_nh, robot_hw_nh)) {
      return false;
    }
    robot_hw_nh.getParam("power_limit", powerLimit_);
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x01,.mst_id=0x11});
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x02,.mst_id=0x12});
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x03,.mst_id=0x13});
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x04,.mst_id=0x14});
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x05,.mst_id=0x15});
    CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x06,.mst_id=0x16});

    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x01,.mst_id=0x11});
    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x02,.mst_id=0x12});
    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x03,.mst_id=0x13});
    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x04,.mst_id=0x14});
    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x05,.mst_id=0x15});
    CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x06,.mst_id=0x16});
    motorsInterface = std::make_shared<damiao::Motor_Control>("can0",&this->CAN1,damiao::canfd);
    motorsInterface2 = std::make_shared<damiao::Motor_Control>("can1",&this->CAN2,damiao::canfd);

    setupJoints();
    setupImu();
    setupContactSensor(robot_hw_nh);

    std::string robot_type;
    root_nh.getParam("robot_type", robot_type);
    
    //joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
    //contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(std::string("/contact"), 10);
    return true;
}

void DmHW::read(const ros::Time& time, const ros::Duration& period) 
{
  //LF
  for (int i = 0; i < 3; ++i) {
    jointData_[i].pos_ =motorsInterface->getMotor(i+1)->Get_Position();
    jointData_[i].vel_ =motorsInterface->getMotor(i+1)->Get_Velocity();
    jointData_[i].tau_=motorsInterface->getMotor(i+1)->Get_tau();
  }
  //LH
  for (int i = 3; i < 6; ++i) {
    jointData_[i].pos_ =motorsInterface2->getMotor(i-2)->Get_Position() * directionMotor_LeftH[i-3];
    jointData_[i].vel_ =motorsInterface2->getMotor(i-2)->Get_Velocity() * directionMotor_LeftH[i-3];
    jointData_[i].tau_ =motorsInterface2->getMotor(i-2)->Get_tau() * directionMotor_LeftH[i-3];
  }
  //RF 
  for (int i=6; i < 9; ++i) {
    jointData_[i].pos_ =motorsInterface->getMotor(i-2)->Get_Position() * directionMotor_RightF[i-6];
    jointData_[i].vel_ =motorsInterface->getMotor(i-2)->Get_Velocity() * directionMotor_RightF[i-6];
    jointData_[i].tau_ =motorsInterface->getMotor(i-2)->Get_tau() * directionMotor_RightF[i-6];
  }
  //RH
  for (int i=9; i < 12; ++i) {
    jointData_[i].pos_ =motorsInterface2->getMotor(i-5)->Get_Position() * directionMotor_RightH[i-9];
    jointData_[i].vel_ =motorsInterface2->getMotor(i-5)->Get_Velocity() * directionMotor_RightH[i-9];
    jointData_[i].tau_ =motorsInterface2->getMotor(i-5)->Get_tau() * directionMotor_RightH[i-9];
  }
  std_msgs::Float64MultiArray time_msg;
  time_msg.data.resize(6);
  for (size_t i = 0; i < 6; ++i)
  {
    time_msg.data[i] = motorsInterface->getMotor(i+1)->getTimeInterval();
  }

  imuData_.ori[0] = yesenceIMU_.orientation.x;
  imuData_.ori[1] = yesenceIMU_.orientation.y;
  imuData_.ori[2] = yesenceIMU_.orientation.z;
  imuData_.ori[3] = yesenceIMU_.orientation.w;
  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;
  // std::cerr<<"x: "<<imuData_.ori[0]<<" "
  //       <<"y: "<<imuData_.ori[1]<<" "
  //       <<"z: "<<imuData_.ori[2]<<" "
  //       <<"w: "<<imuData_.ori[3]<<" "<<std::endl;
    // std::cerr<<"pos1: "<<jointData_[0].pos_<<
    //     "   pos2: "<<jointData_[1].pos_<<
    //     "   pos3: "<<jointData_[2].pos_<<
    //     "   pos4: "<<jointData_[3].pos_<<
    //     "   pos5: "<<jointData_[4].pos_<<
    //     "   pos6: "<<jointData_[5].pos_<<
    //     "   pos7: "<<jointData_[6].pos_<<
    //     "   pos8: "<<jointData_[7].pos_<<
    //     "   pos9: "<<jointData_[8].pos_<<
    //     "   pos10: "<<jointData_[9].pos_<<
    //     "   pos11: "<<jointData_[10].pos_<<
    //     "   pos12: "<<jointData_[11].pos_<<
    //     std::endl;
  
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  //updateJoystick(time);
  //updateContact(time);
}

void DmHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) 
{
  for (int i = 0; i < 12; ++i) {
    jointData_[i].pos_des_ = 0.0 ;
    jointData_[i].vel_des_ = 0.0 ;
    jointData_[i].ff_ = 0.0;
    jointData_[i].kp_ = 0.0;
    jointData_[i].kd_ = 0.0;
}

  //LF
  for (int i = 0; i < 3; ++i) {
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ ;
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_ ;
    dmSendcmd_[i].ff_ = jointData_[i].ff_;
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    // motorsInterface->control_mit(*motorsInterface->getMotor(i+1),0.0,0.0,  0.0,0.0, 0.0);
    motorsInterface->control_mit(*motorsInterface->getMotor(i+1),dmSendcmd_[i].kp_,dmSendcmd_[i].kd_ ,
                                        dmSendcmd_[i].pos_des_,dmSendcmd_[i].vel_des_, dmSendcmd_[i].ff_);
  }
  //LH
  for (int i = 3; i < 6; ++i) {
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * directionMotor_LeftH[i-3];
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * directionMotor_LeftH[i-3];
    dmSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_LeftH[i-3];
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    // motorsInterface2->control_mit(*motorsInterface2->getMotor(i-2),0.0,0.0,  0.0,0.0, 0.0);
     motorsInterface2->control_mit(*motorsInterface2->getMotor(i-2),dmSendcmd_[i].kp_,dmSendcmd_[i].kd_ ,
                                        dmSendcmd_[i].pos_des_,dmSendcmd_[i].vel_des_, dmSendcmd_[i].ff_);
  }
  //RF
  for (int i = 6; i < 9; ++i) {
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * directionMotor_RightF[i-6];
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * directionMotor_RightF[i-6];
    dmSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_RightF[i-6];
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    // motorsInterface->control_mit(*motorsInterface->getMotor(i-2),0.0,0.0,  0.0,0.0, 0.0);
    motorsInterface->control_mit(*motorsInterface->getMotor(i-2),dmSendcmd_[i].kp_,dmSendcmd_[i].kd_ ,
                                        dmSendcmd_[i].pos_des_,dmSendcmd_[i].vel_des_, dmSendcmd_[i].ff_);
  }
  //RH
  for (int i = 9; i < 12; ++i) {
    dmSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * directionMotor_RightH[i-9];
    dmSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * directionMotor_RightH[i-9];
    dmSendcmd_[i].ff_ = jointData_[i].ff_ * directionMotor_RightH[i-9];
    dmSendcmd_[i].kp_ = jointData_[i].kp_;
    dmSendcmd_[i].kd_ = jointData_[i].kd_;
    // motorsInterface2->control_mit(*motorsInterface2->getMotor(i-5),0.0,0.0,  0.0,0.0, 0.0);
    motorsInterface2->control_mit(*motorsInterface2->getMotor(i-5),dmSendcmd_[i].kp_,dmSendcmd_[i].kd_ ,
                                        dmSendcmd_[i].pos_des_,dmSendcmd_[i].vel_des_, dmSendcmd_[i].ff_);
  }

}

bool DmHW::setupJoints() {
  std::map<std::string, int> joint_to_index_map = {
    {"LF_HAA", 0}, {"LF_HFE", 1}, {"LF_KFE", 2},
    {"RF_HAA", 6}, {"RF_HFE", 7}, {"RF_KFE", 8},
    {"LH_HAA", 3}, {"LH_HFE", 4}, {"LH_KFE", 5},
    {"RH_HAA", 9}, {"RH_HFE", 10}, {"RH_KFE", 11}
  };
  for (const auto& joint : urdfModel_->joints_) {
    auto it = joint_to_index_map.find(joint.first);
    if (it == joint_to_index_map.end()) {
      //ROS_WARN("Joint %s not found in mapping table, skipping", joint.first.c_str());
      continue;//重要，不然初始化了空指针
    }
    
    int index = it->second;
    DmMotorData* motorPtr = nullptr;
    hardware_interface::JointStateHandle state_handle(joint.first, 
          &jointData_[index].pos_, 
          &jointData_[index].vel_,
          &jointData_[index].tau_);
      jointStateInterface_.registerHandle(state_handle);
      hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, 
          &jointData_[index].pos_des_, 
          &jointData_[index].vel_des_,
          &jointData_[index].kp_, 
          &jointData_[index].kd_, 
          &jointData_[index].ff_));
      motorPtr = &jointData_[index];
    jointNameToMotorData_[joint.first] = motorPtr;
  }
  return true;
}

bool DmHW::setupImu() {
  imuSensorInterface_.registerHandle(
      hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel,
                                          imuData_.angular_vel_cov, imuData_.linear_acc, imuData_.linear_acc_cov));

  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;
  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

bool DmHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

// void DmHW::updateJoystick(const ros::Time& time) {
// }

// void DmHW::updateContact(const ros::Time& time) {
//   if ((time - lastContactPub_).toSec() < 1 / 50.) {
//     return;
//   }
//   lastContactPub_ = time;
//   std_msgs::Int16MultiArray contactMsg;
//   for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
//     // contactMsg.data.push_back(estimatedContactState_[i] ? 100 : 0);  // 使用估计的接触状态
//   }
//   contactPublisher_.publish(contactMsg);
// }

}  // namespace legged