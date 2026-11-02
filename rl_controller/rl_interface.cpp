/*
 * @Author: Peixin Ma peixin.ma@hlrobot.tech
 * @Date: 2024-07-29 11:47:25
 * @LastEditors: Peixin Ma peixin.ma@hlrobot.tech
 * @LastEditTime: 2025-07-22
 * @FilePath: /rl_controller/rl_controller/rl_interface.cpp
 * @Description: RL interface — New Leg-AMP (22-action output, leg+arm mapping)
 */
#include "rl_interface.hpp"

namespace rl
{
  RlInterface::RlInterface()
  {
    rlController = std::make_shared<RLController>();
    // 22-dim output: 12 legs + 10 arms (upper_left1~5, upper_right1~5)
    _rlOutputData = std::make_shared<RLOutputData>(12, 0);
    _statePercent = 0.0;
    _armPercent = 0.0;
    _stateTorquePercent = 0.0;
    _stateDuration = 2000.0;
    _isFirstZeroState = true;
    _isStarthWalkState = false;
    _isSwingingArm = true;
    arm_joint_roll_target = 0.35;
    rlController->init();
  }

  /**
   * @description: update rl interface using message::inputStruct and message::outputStruct
   */
  void RlInterface::update(const message::inputStruct &inputStruct,
                            message::outputStruct &outputStruct)
  {
    getJointData(inputStruct);
    getImuData(inputStruct);
    getCmdVel(inputStruct);

    // Publish one complete joint/IMU/command frame for the 50 Hz policy.
    rlController->commitInputSnapshot();

    if (robot_mode == 1) // DAMPING mode
    {
      reEnterAMP = true;
      if (_isStarthWalkState)
      {
        _stateTorquePercent += 1.0 / (200.0);
        _stateTorquePercent = std::min(_stateTorquePercent, double(1));
        rlController->handleSoftBackMode(_stateTorquePercent,
                                          rlController->_rlInputData.jointInputData.jointTorque,
                                          _lastTorque,
                                          _rlOutputData->jointOutputData);
        for (int i = 0; i < robot::gLegJointNum; ++i)
        {
          if (i == 4 || i == 5 || i == 10 || i == 11)
          {
            outputStruct.leg_cmd[i].pos_des = 0.0;
            outputStruct.leg_cmd[i].vel_des = 0.0;
            outputStruct.leg_cmd[i].torq_des = 0.0;
            outputStruct.leg_cmd[i].kp = 0.0;
            outputStruct.leg_cmd[i].kd = 15.0;
          }
          else
          {
            outputStruct.leg_cmd[i].pos_des = 0.0;
            outputStruct.leg_cmd[i].vel_des = 0.0;
            outputStruct.leg_cmd[i].torq_des = 0.0;
            outputStruct.leg_cmd[i].kp = 0.0;
            outputStruct.leg_cmd[i].kd = 30.0;
          }
        }
        for (int i = 0; i < robot::gArmJointNum; i++)
        {
          outputStruct.arm_cmd[i].pos_des = 0.0;
          outputStruct.arm_cmd[i].vel_des = 0.0;
          outputStruct.arm_cmd[i].kp = 0.0;
          outputStruct.arm_cmd[i].kd = 10.0;
          outputStruct.arm_cmd[i].torq_des = 0.0;
        }
        for (int i = 0; i < robot::gHeadJointNum; i++)
        {
          outputStruct.head_cmd[i].pos_des = 0.0;
          outputStruct.head_cmd[i].vel_des = 0.0;
          outputStruct.head_cmd[i].kp = 0.0;
          outputStruct.head_cmd[i].kd = 10.0;
          outputStruct.head_cmd[i].torq_des = 0.0;
        }
        outputStruct.waist_cmd[0].pos_des = 0.0;
        outputStruct.waist_cmd[0].vel_des = 0.0;
        outputStruct.waist_cmd[0].kp = 0.0;
        outputStruct.waist_cmd[0].kd = 10.0;
        outputStruct.waist_cmd[0].torq_des = 0.0;
      }
      else if (_statePercent < 1 && _isFirstZeroState == true)
      {
        rlController->handleZeroMode(_statePercent,
                                      rlController->_rlInputData.jointInputData.jointPos,
                                      _rlOutputData->jointOutputData);
        _statePercent += 1 / _stateDuration;
        _statePercent = std::min(_statePercent, double(1));
        setJointCmd(inputStruct, outputStruct);
      }
      else
      {
        rlController->handleZeroMode(1,
                                      rlController->_rlInputData.jointInputData.jointPos,
                                      _rlOutputData->jointOutputData);
        _statePercent = 0;
        _isFirstZeroState = false;
        setJointCmd(inputStruct, outputStruct);
      }
    }
    else if (robot_mode == 7) // RL CONTROL mode
    {
      switch (currentRobotState)
      {
      case RobotState::STAND:
      case RobotState::WALKING:
      {
        if (reEnterAMP)
        {
          rlController->_rlLoopCount = 0;
          rlController->resetJointTargetInterpolation(
              rlController->_rlInputData.jointInputData.jointPos);
          reEnterAMP = false;
        }
        rlController->stand_command = true;
        const bool policyOutputReady =
            rlController->handleWalkMode(_rlOutputData->jointOutputData);
        if (!policyOutputReady)
        {
          // The inference thread starts asynchronously. Until its first valid
          // output arrives, hold the measured policy-joint positions instead
          // of publishing the zero-initialized output buffer.
          rlController->handleZeroMode(
              0.0,
              rlController->_rlInputData.jointInputData.jointPos,
              _rlOutputData->jointOutputData);
        }
        // Save last torques for all 12 leg joints.
        for (int i = 0; i < robot::gLegJointNum; i++)
        {
          _lastTorque[i] = outputStruct.leg_cmd[i].torq_des;
        }
        _isStarthWalkState = true;
        setJointCmd(inputStruct, outputStruct);
        break;
      }
      default:
        break;
      }
    }
  }

  /**
   * @description: get joint data from real robot via message::inputStruct
   *               Reads 12 leg joints + 10 arm joints (upper_1~5 each side)
   */
  void RlInterface::getJointData(const message::inputStruct &inputStruct)
  {
    // Leg joints only: indices 0-11 (leg_l1~l6, leg_r1~r6)
    for (int i = 0; i < robot::gLegJointNum; ++i)
    {
      rlController->_rlInputData.jointInputData.jointPos[i] =
          inputStruct.leg_data[i].pos_real;
      rlController->_rlInputData.jointInputData.jointVel[i] =
          inputStruct.leg_data[i].vel_real;
      rlController->_rlInputData.jointInputData.jointTorque[i] =
          inputStruct.leg_data[i].torq_real;
    }
  }

  /**
   * @description: get imu data from real robot
   */
  void RlInterface::getImuData(const message::inputStruct &inputStruct)
  {
    // Quaternion: wxyz order
    rlController->_rlInputData.quat =
        Eigen::Quaterniond(inputStruct.orientation[0],  // w
                           inputStruct.orientation[1],  // x
                           inputStruct.orientation[2],  // y
                           inputStruct.orientation[3]); // z

    // Euler angles: rpy
    rlController->_rlInputData.rpy =
        Eigen::Vector3d(inputStruct.rpy[0], inputStruct.rpy[1], inputStruct.rpy[2]);

    // Angular velocity: gyro
    rlController->_rlInputData.angVel =
        Eigen::Vector3d(inputStruct.gyr[0], inputStruct.gyr[1], inputStruct.gyr[2]);
  }

  /**
   * @description: Get velocity commands
   */
  void RlInterface::getCmdVel(const message::inputStruct &inputStruct)
  {
    rlController->_rlInputData.baseCmdVel[0] = std::clamp(inputStruct.cmdVx, -0.5, 0.8);
    rlController->_rlInputData.baseCmdVel[1] = std::clamp(inputStruct.cmdVy, -0.01, 0.01);
    rlController->_rlInputData.baseCmdVel[2] = std::clamp(inputStruct.cmdWz, -1.0, 1.0);
  }

  /**
   * @description: Map the 22-dim RL output to leg and arm joint commands
   *
   *   22-action output mapping:
   *     [0:12]  → leg_cmd[0:12]    (leg_l1~l6, leg_r1~r6)
   *     [12:17] → arm_cmd[0:5]     (upper_left1~5, includes wrist yaw)
   *     [17:22] → arm_cmd[7:12]    (upper_right1~5, includes wrist yaw)
   *
   *   Unchanged joints (held in position):
   *     arm_cmd[5:7]   — left wrist pitch/roll (not in action set)
   *     arm_cmd[12:14] — right wrist pitch/roll (not in action set)
   *   Head, waist, hands → zero torque
   */
  void RlInterface::setJointCmd(const message::inputStruct &inputStruct,
                                 message::outputStruct &outputStruct)
  {
    // ── Leg joints [0:12] → leg_cmd[0:12] ──
    for (int i = 0; i < robot::gLegJointNum; ++i)
    {
      outputStruct.leg_cmd[i].pos_des = _rlOutputData->jointOutputData.jointPosDes[i];
      outputStruct.leg_cmd[i].vel_des = 0.0;
      outputStruct.leg_cmd[i].torq_des =
          _rlOutputData->jointOutputData.kp[i] *
              (_rlOutputData->jointOutputData.jointPosDes[i] -
               inputStruct.leg_data[i].pos_real) +
          _rlOutputData->jointOutputData.kd[i] *
              (_rlOutputData->jointOutputData.jointVelDes[i] -
               inputStruct.leg_data[i].vel_real);
      outputStruct.leg_cmd[i].kp = _rlOutputData->jointOutputData.kp[i];
      outputStruct.leg_cmd[i].kd = _rlOutputData->jointOutputData.kd[i];
    }

    // ── Arm swing: only the shoulder (arm joint 1) follows the knee difference ──
    // Left knee = leg_l4 = leg_data[3], right knee = leg_r4 = leg_data[9].
    // sim2sim: target[13]=+0.5*knee_diff (left shoulder), target[18]=-0.5*knee_diff (right shoulder).
    const double kneeDiff =
        inputStruct.leg_data[3].pos_real - inputStruct.leg_data[9].pos_real;

    // 手腕 pitch/roll 默认 0，其余手臂关节从 YAML default_joint_angle 读取。
    for (int i = 0; i < robot::gArmJointNum; i++)
    {
      outputStruct.arm_cmd[i].pos_des = 0.0;
      outputStruct.arm_cmd[i].vel_des = 0.0;
      outputStruct.arm_cmd[i].kp = 100.0;
      outputStruct.arm_cmd[i].kd = 5.0;
      outputStruct.arm_cmd[i].torq_des = 0.0;
    }
    // 非肩关节（upper_2~5）保持 default_joint_angle 设定
    outputStruct.arm_cmd[1].pos_des  = rlController->getDefaultJointAngle("upper_left2_joint");
    outputStruct.arm_cmd[2].pos_des  = rlController->getDefaultJointAngle("upper_left3_joint");
    outputStruct.arm_cmd[3].pos_des  = rlController->getDefaultJointAngle("upper_left4_joint");
    outputStruct.arm_cmd[4].pos_des  = rlController->getDefaultJointAngle("upper_left5_joint");
    outputStruct.arm_cmd[8].pos_des  = rlController->getDefaultJointAngle("upper_right2_joint");
    outputStruct.arm_cmd[9].pos_des  = rlController->getDefaultJointAngle("upper_right3_joint");
    outputStruct.arm_cmd[10].pos_des = rlController->getDefaultJointAngle("upper_right4_joint");
    outputStruct.arm_cmd[11].pos_des = rlController->getDefaultJointAngle("upper_right5_joint");
    // 肩关节（joint 1）摆动 + 默认值
    outputStruct.arm_cmd[0].pos_des = 0.5 * kneeDiff + rlController->getDefaultJointAngle("upper_left1_joint");   // upper_left1  shoulder
    outputStruct.arm_cmd[7].pos_des = -0.5 * kneeDiff + rlController->getDefaultJointAngle("upper_right1_joint"); // upper_right1 shoulder

    // ── Head → zero torque ──
    for (int i = 0; i < robot::gHeadJointNum; i++)
    {
      outputStruct.head_cmd[i].pos_des = 0.0;
      outputStruct.head_cmd[i].vel_des = 0.0;
      outputStruct.head_cmd[i].kp = 0.0;
      outputStruct.head_cmd[i].kd = 10.0;
      outputStruct.head_cmd[i].torq_des = 0.0;
    }

    // ── Waist → zero torque ──
    outputStruct.waist_cmd[0].pos_des = 0.0;
    outputStruct.waist_cmd[0].vel_des = 0.0;
    outputStruct.waist_cmd[0].kp = rlController->_waistKp;
    outputStruct.waist_cmd[0].kd = rlController->_waistKd;
    outputStruct.waist_cmd[0].torq_des = 0.0;

    // ── Hands → zero torque ──
    for (int i = 0; i < robot::gHandJointNum; i++)
    {
      outputStruct.hand_cmd[i].pos_des = 0.0;
      outputStruct.hand_cmd[i].vel_des = 0.0;
      outputStruct.hand_cmd[i].kp = 0.0;
      outputStruct.hand_cmd[i].kd = 10.0;
      outputStruct.hand_cmd[i].torq_des = 0.0;
    }
  }

} // namespace rl
