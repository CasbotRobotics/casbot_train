#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <common/joint_info/joint_info.hpp>

namespace message
{

  struct JointCmd
  {
    JointCmd()
    {
      pos_des = 0.0;
      vel_des = 0.0;
      torq_des = 0.0;
      kp = 0;
      kd = 0;
    }
    double pos_des;
    double vel_des;
    double torq_des;
    double kp;
    double kd;
  };

  struct JointData
  {
    JointData()
    {
      pos_real = 0.0;
      vel_real = 0.0;
      torq_real = 0.0;
    }
    double pos_real;
    double vel_real;
    double torq_real;
  };

  struct inputStruct
  {
    double cmdVx, cmdVy, cmdWz;
    double rpy[3], gyr[3], acc[3], orientation[4];    // rpy gyr acc : xyz ; orientation:wxyz
    
    JointData leg_data[robot::gLegJointNum];
    JointData arm_data[robot::gArmJointNum];
    JointData waist_data[robot::gWaistJointNum];
    JointData head_data[robot::gHeadJointNum];
    JointData hand_data[robot::gHandJointNum];
  };

  struct outputStruct
  {
    JointCmd leg_cmd[robot::gLegJointNum];
    JointCmd arm_cmd[robot::gArmJointNum];
    JointCmd waist_cmd[robot::gWaistJointNum];
    JointCmd head_cmd[robot::gHeadJointNum];
    JointCmd hand_cmd[robot::gHandJointNum];
  };
} // namespace message
