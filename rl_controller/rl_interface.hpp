/*
 * @Author: Peixin Ma peixin.ma@hlrobot.tech
 * @Date: 2024-07-29 11:47:25
 * @LastEditors: Peixin Ma peixin.ma@hlrobot.tech
 * @LastEditTime: 2024-12-17 16:29:21
 * @FilePath: /hl_motion/hl_core/module/hl_rl/src/rl_interface.hpp
 * @Description: RL interface
 */
#pragma once
#include <vector>
#include "rl_controller.hpp"
#include <common/common.hpp>

namespace rl
{

  enum class RobotState
  {
    STAND = 3,
    WALKING = 4,
  };

  class RlInterface
  {
  public:
    RlInterface();
    ~RlInterface() = default;
    int robot_mode = 0;

    void update(const message::inputStruct &inputStruct, message::outputStruct &outputStruct);
  
  private:
    void getJointData(const message::inputStruct &inputStruct);
    void getImuData(const message::inputStruct &inputStruct);
    void setJointCmd(const message::inputStruct &inputStruct, message::outputStruct &outputStruct);
    void getCmdVel(const message::inputStruct &inputStruct);
    
  private:
    std::shared_ptr<RLController> rlController;
    std::shared_ptr<RLOutputData> _rlOutputData;
    RobotState currentRobotState{RobotState::STAND};


  private:
    bool _isFirstZeroState;
    bool _isStarthWalkState;
    bool _isSwingingArm;
    
    double _lastTorque[12];  // 12 legs only
    bool reEnterAMP{true};

  private:
    double _statePercent;
    double _stateTorquePercent;
    double _stateDuration;
    double _armPercent;

    double arm_joint_roll_target = 0.35;
  };

} // namespace rl
