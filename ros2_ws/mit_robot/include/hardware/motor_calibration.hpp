// Shared calibration parser for hardware and hardware-in-the-loop simulation.
#ifndef MYMIT_ROBOT_HARDWARE_MOTOR_CALIBRATION_HPP_
#define MYMIT_ROBOT_HARDWARE_MOTOR_CALIBRATION_HPP_
#include <fstream>
#include <sstream>
#include <string>
#include "hardware/dm1_mit_interface.hpp"
namespace dm1_hardware
{
inline Dm1MitInterface::CalibrationArray loadCalibration(
  const std::string & path)
{
  // 标定数组下标必须和 RobotRunner/DM1 模型的 12 关节顺序一致。
  constexpr std::array<const char *, kNumJoints> expected_names{
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf"};

  std::ifstream input(path);
  if (!input) {throw std::runtime_error("无法打开标定文件: " + path);}
  dm1_hardware::Dm1MitInterface::CalibrationArray result{};
  std::string line;
  std::size_t index = 0;
  // 每个有效数据行对应一个电机；空行和 # 注释行会被跳过。
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {continue;}
    if (index >= result.size()) {
      throw std::runtime_error("标定文件包含超过 12 个电机");
    }
    std::istringstream values(line);
    std::string name;
    std::string bus_name;
    std::string can_id_text;
    std::string master_id_text;
    unsigned int can_id = 0;
    unsigned int master_id = 0;
    int direction = 0;
    float zero = 0.0F;
    std::string extra;
    // 标定文件每行格式：关节名 总线 物理CAN ID Master ID 方向 零位(rad)。
    if (!(values >> name >> bus_name >> can_id_text >> master_id_text >> direction >> zero) ||
      (values >> extra))
    {
      throw std::runtime_error(
              "标定文件每行格式必须为: 关节名 总线 CAN_ID MASTER_ID 方向 零位_RAD");
    }
    // 不仅检查关节名，还检查它在文件中的固定顺序。
    if (name != expected_names[index]) {
      throw std::runtime_error(
              "标定文件第 " + std::to_string(index + 1) + " 行的关节顺序不正确");
    }
    std::uint8_t bus = 0;
    // 将可读的总线名称转换成驱动使用的数值编号。
    if (bus_name == "can0" || bus_name == "0") {
      bus = 0;
    } else if (bus_name == "can1" || bus_name == "1") {
      bus = 1;
    } else {
      throw std::runtime_error("电机总线必须是 can0/0 或 can1/1");
    }
    try {
      // 支持十进制和十六进制的 CAN/master ID。
      can_id = std::stoul(can_id_text, nullptr, 0);
      master_id = std::stoul(master_id_text, nullptr, 0);
    } catch (const std::exception &) {
      throw std::runtime_error("CAN ID 和 master ID 必须是整数");
    }
    // 物理 CAN ID 使用 DM1 的 4-bit 地址范围；master ID 使用 11-bit 范围。
    if (can_id == 0 || can_id > 0x0F || master_id == 0 || master_id > 0x7FF) {
      throw std::runtime_error(
              "物理 CAN ID 必须在 [1,0x0f] 范围内，master ID 必须在 [1,0x7ff] 范围内");
    }
    result[index] = dm1_hardware::MotorCalibration{
      dm1_hardware::MotorAddress{
        bus, static_cast<std::uint16_t>(can_id), static_cast<std::uint16_t>(master_id)},
      direction, zero};
    // 下一个有效行写入下一个关节槽位。
    ++index;
  }
  // 必须恰好提供 12 个电机，不能少也不能多。
  if (index != result.size()) {
    throw std::runtime_error("标定文件必须恰好包含 12 个电机");
  }
  return result;
}
}  // namespace dm1_hardware
#endif
