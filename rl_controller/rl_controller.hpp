/*
 * @Author: Peixin Ma peixin.ma@hlrobot.tech
 * @Date: 2024-07-29 11:47:25
 * @LastEditors: Peixin Ma peixin.ma@hlrobot.tech
 * @LastEditTime: 2024-12-20 16:39:54
 * @FilePath: /hl_motion/hl_core/module/hl_rl/src/rl_controller.hpp
 * @Description: RL controller inherits from RLBase
 */
#pragma once

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>


#include <common/common.hpp>
#include "rl_struct.hpp"
#include "loco_observation.hpp"


namespace rl
{
  class RLController
  {
  public:
    RLController();
    virtual ~RLController()
    {
      _stopThread.store(true);
      if (_rl_thread && _rl_thread->joinable())
      {
        _rl_thread->join();
      }
    }
    void init();

    void handleSoftBackMode(const double stateTorquePercent, std::vector<float> &currentJointTor, double lastJointTor[], JointOutputData &rlOutputData);
    void handleZeroMode(const double standPercent, std::vector<float> &currentJointPos, JointOutputData &rlOutputData);
    bool handleWalkMode(JointOutputData &rlOutputData);
    void commitInputSnapshot();
    void resetJointTargetInterpolation(const std::vector<float> &currentJointPos);
    double getDefaultJointAngle(const std::string &name) const;
    
    RLInPutData _rlInputData;
    int64_t _rlLoopCount;
    bool stand_command{true};

  private:
    void loadModel(const std::string &model_path);
    void loadRLCfg();
    void creatThreadFirst(size_t dim);
    void threadRunning(size_t dim);
    bool updateJointOutputData(JointOutputData &jointOutputData);
    void beginJointTargetInterpolation(const JointOutputData &newTarget);
    void evaluateJointTargetInterpolation(
        const std::chrono::steady_clock::time_point &now,
        std::vector<float> &position) const;
    void handleEnd();
    void useRlAmpModel(JointOutputData &rlOutputData);
    bool updateRLData();
    void computeAmpObservation();
    std::vector<float> computeActions(const std::vector<float> &observations);
    void computeJointDes(const std::vector<float> &actions, JointOutputData &rlOutputData);
    void adjustStandingPosture(std::vector<float> &actions);

  private:
    int64_t _loopCount;
    int _actionsSize;
    int _observationSize;
    RLRobotCfg _robotCfg{};
    Command _command;
    Proprioception _propri;
    int _gaitPhaseFrames{0}; // 180: none, 184: current, 196: four-frame history
    loco::GaitPhaseHistory _gaitPhaseHistory;
    // Owned by the policy thread. Advance even at zero command, as in sim2sim.
    double _gaitPhaseTimeSec{0.0};
    std::atomic<bool> _resetGaitPhase{false};
      
  private:
    Eigen::Matrix<double, Eigen::Dynamic, 1> _lastActions;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _standJointAngles;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _defaultJointAngles;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _zeroJointAngles;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _actionScale;           // per-joint action scale (22-dim)
    Eigen::Matrix<double, Eigen::Dynamic, 1> _defaultObsJointAngles; // leg-only default joint angles for obs (12-dim)

  private:
    std::string _modelFilePath;
    std::string _rlMode; // "rl" or "amp"
    std::shared_ptr<Ort::Env> _onnxEnvPtr;
    std::unique_ptr<Ort::Session> _sessionPtr;
    std::vector<const char *> inputNames_;
    std::vector<const char *> outputNames_;
    std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings;
    std::vector<std::vector<int64_t>> inputShapes_;
    std::vector<std::vector<int64_t>> outputShapes_;
    bool _isfirstRecObs{true};
    int _historySize;
    int _inputSize;

    bool _isStatic{false};
    bool _changingPosture{false};
    bool _adjustLeftLeg{false};
    float _targetActionHip{0.0f};
    float _targetActionKnee{0.0f};
    float _targetPosHip{0.0f};
    float hipDiff{0.0f};
    int _staticCounter{0};
    bool _forwardCommand{false};
    bool _inPosition{false};
    int _postAdjustCounter{0};
    bool _enable_standing_posture_adjustment{true};
    int _standing_posture_static_threshold{1000};
    float _standing_posture_tolerance{4.0f}; // degree

    Ort::MemoryInfo _memoryInfo;
    Eigen::Matrix<float, Eigen::Dynamic, 1> _proprioHistoryBuffer;

    std::vector<float> _actions;
    std::vector<float> _observations;

    // Complete input-frame handoff from the 1 kHz interface to the
    // 50 Hz policy thread.
    std::mutex _inputSnapshotMutex;
    RLInPutData _pendingInputSnapshot;
    RLInPutData _policyInputSnapshot;
    bool _hasInputSnapshot{false};

    // Disable the policy-rate low-pass filter for an A/B test against the
    // smoothing performed by the downstream motion layer.
    bool _isActionsFilter{false};

    std::shared_ptr<std::thread> _rl_thread = nullptr;
    std::atomic<bool> _stopThread{false};

    std::vector<JointOutputData> _outputBuffers;
    std::atomic<int> _readIndex{0};
    std::atomic<int> _writeIndex{1};
    std::atomic<bool> _newDataAvailable{false};
    int _noDataCount{0};
    bool _hasPolicyOutput{false};

    // Band-style target interpolation. The policy action and last_action stay
    // at policy rate; only the final desired joint positions are interpolated
    // by the 1 kHz control loop before PD is evaluated.
    bool _enableJointTargetInterpolation{false};
    double _jointTargetInterpolationDurationSec{0.02};
    JointOutputData _latestPolicyTarget;
    std::vector<float> _jointTargetInterpolationStart;
    std::vector<float> _jointTargetInterpolationEnd;
    std::vector<float> _jointTargetInterpolationInitial;
    std::chrono::steady_clock::time_point _jointTargetInterpolationStartTime{};
    bool _jointTargetInterpolationActive{false};
    bool _jointTargetInterpolationHasInitial{false};

  private:
    double imuRollAngleBias_ = 0.0;
    double imuPitchAngleBias_ = 0.0;
    double imuYawAngleBias_ = 0.0;

  public:
    int _actuatedDofNum;
    int _obsActuatorDofNum;

    double _cmd_vel_bias_x = 0.0;
    double _cmd_vel_bias_y = 0.0;
    double _cmd_vel_bias_yaw = 0.0;
    bool _enableCmdVelSlewX = true;
    // Maximum change of the forward command seen by the policy, in m/s^2.
    // 1.0 ramps 0 -> 0.5 m/s in approximately 0.5 s at any policy rate.
    double _cmdVelSlewRateX = 1.0;
    double _rampedCmdVelX = 0.0;
    float _waistKp = 500.0f;
    float _waistKd = 10.0f;
  };

} // namespace rl
