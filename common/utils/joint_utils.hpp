#pragma once

#include <string>
#include <vector>

#include <common/common.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace robot
{

// sensor_msgs::msg::JointState
bool getJointDataFromOutputStruct(const message::outputStruct &outputStruct, sensor_msgs::msg::JointState& joint_data);
bool getPdGainsFromOutputStruct(const message::outputStruct &outputStruct,
                                std::vector<double> &kp,
                                std::vector<double> &kd);
bool setJointDataInInputStruct(const sensor_msgs::msg::JointState &joint_state, message::inputStruct &inputStruct);

// sensor_msgs::msg::Imu
bool setImuDataInInputStruct(const sensor_msgs::msg::Imu &imu_msg, message::inputStruct &inputStruct);

} // namespace robot
