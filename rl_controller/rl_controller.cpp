/*
 * @Author: Peixin Ma peixin.ma@hlrobot.tech
 * @Date: 2024-07-29 11:47:25
 * @LastEditors: Peixin Ma peixin.ma@hlrobot.tech
 * @LastEditTime: 2025-07-22
 * @FilePath: /rl_controller/rl_controller/rl_controller.cpp
 * @Description: 12-action loco policy, 45x4 history + optional gait-phase history
 */
#include "rl_controller.hpp"

#include <stdexcept>

namespace rl
{
  constexpr double kDegToRad = M_PI / 180.0;

  RLController::RLController()
    : _rlInputData(robot::gLegJointNum, robot::gArmJointNum)
    , _memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    , _pendingInputSnapshot(robot::gLegJointNum, robot::gArmJointNum)
    , _policyInputSnapshot(robot::gLegJointNum, robot::gArmJointNum)
  {
  }

  /**
   * @description: load RL config from yaml and load model from onnx file
   */
  void RLController::init()
  {
    _actuatedDofNum = 12;  // 12 legs only
    _obsActuatorDofNum = 12; // leg-only feedback

    if (_actuatedDofNum <= 0)
    {
      motion::Logger::instance().error("Invalid actuated dof num: {}", _actuatedDofNum);
      return;
    }
    _loopCount = 0;
    _rlLoopCount = 0;

    _outputBuffers.clear();
    _outputBuffers.reserve(2);
    _outputBuffers.emplace_back(_actuatedDofNum);  // 22-dim
    _outputBuffers.emplace_back(_actuatedDofNum);
    _readIndex.store(0, std::memory_order_relaxed);
    _writeIndex.store(1, std::memory_order_relaxed);
    _newDataAvailable.store(false, std::memory_order_relaxed);
    _hasPolicyOutput = false;
    _stopThread.store(false, std::memory_order_relaxed);

    _latestPolicyTarget = JointOutputData(_actuatedDofNum);
    _jointTargetInterpolationStart.assign(_actuatedDofNum, 0.0f);
    _jointTargetInterpolationEnd.assign(_actuatedDofNum, 0.0f);
    _jointTargetInterpolationInitial.assign(_actuatedDofNum, 0.0f);
    _jointTargetInterpolationActive = false;
    _jointTargetInterpolationHasInitial = false;

    loadRLCfg();
    motion::Logger::instance().info("RL Controller loaded config file: {}", _modelFilePath);
    loadModel(_modelFilePath);
    motion::Logger::instance().info("init RL Controller successful.");
  }

  /**
   * @description: Load robot config from yaml
   */
  void RLController::loadRLCfg()
  {
    _enable_standing_posture_adjustment = false;
    _standing_posture_static_threshold = 300;
    _standing_posture_tolerance = 3.0;

    std::string rl_config_file = "/workspace/rl_controller/config/rl_config.yaml";
    YAML::Node config = YAML::LoadFile(rl_config_file);
    motion::Logger::instance().info("RL Controller loaded config file: {}", rl_config_file);

    if (config["LeggedRobotCfg"]["rl_mode"])
    {
      _rlMode = config["LeggedRobotCfg"]["rl_mode"].as<std::string>();
    }
    else
    {
      motion::Logger::instance().error("rl_mode key not found in the YAML configuration file.");
    }

    // get model file from yaml
    _modelFilePath = config["LeggedRobotCfg"][_rlMode]["model_file_path"].as<std::string>();
    motion::Logger::instance().info("model file path: {}", _modelFilePath);

    // get joint angle configs
    _robotCfg.controlCfg.stand_joint_angle =
        config["LeggedRobotCfg"][_rlMode]["init_state"]["stand_joint_angle"]
            .as<std::map<std::string, double>>();
    _robotCfg.controlCfg.default_joint_angle =
        config["LeggedRobotCfg"][_rlMode]["init_state"]["default_joint_angle"]
            .as<std::map<std::string, double>>();

    // get control cfg from yaml
    _robotCfg.controlCfg.stand_stiffness =
        config["LeggedRobotCfg"][_rlMode]["control"]["stand"]["stiffness"]
            .as<std::map<std::string, float>>();
    _robotCfg.controlCfg.stand_damping =
        config["LeggedRobotCfg"][_rlMode]["control"]["stand"]["damping"]
            .as<std::map<std::string, float>>();
    _robotCfg.controlCfg.walk_stiffness =
        config["LeggedRobotCfg"][_rlMode]["control"]["walk"]["stiffness"]
            .as<std::map<std::string, float>>();
    _robotCfg.controlCfg.walk_damping =
        config["LeggedRobotCfg"][_rlMode]["control"]["walk"]["damping"]
            .as<std::map<std::string, float>>();

    _robotCfg.controlCfg.decimation =
        config["LeggedRobotCfg"][_rlMode]["control"]["decimation"].as<int>();
    _robotCfg.controlCfg.dt =
        config["LeggedRobotCfg"][_rlMode]["control"]["dt"].as<float>();
    const YAML::Node enableCmdVelSlewXNode =
        config["LeggedRobotCfg"][_rlMode]["control"]["enable_cmd_vel_slew_x"];
    if (enableCmdVelSlewXNode)
    {
      _enableCmdVelSlewX = enableCmdVelSlewXNode.as<bool>();
    }
    const YAML::Node cmdVelSlewRateXNode =
        config["LeggedRobotCfg"][_rlMode]["control"]["cmd_vel_slew_rate_x"];
    if (cmdVelSlewRateXNode)
    {
      _cmdVelSlewRateX = cmdVelSlewRateXNode.as<double>();
    }

    const YAML::Node enableJointTargetInterpolationNode =
        config["LeggedRobotCfg"][_rlMode]["control"]["enable_joint_target_interpolation"];
    if (enableJointTargetInterpolationNode)
    {
      _enableJointTargetInterpolation =
          enableJointTargetInterpolationNode.as<bool>();
    }

    if (_robotCfg.controlCfg.decimation <= 0 || _robotCfg.controlCfg.dt <= 0.0f)
    {
      throw std::runtime_error("decimation and dt must be positive");
    }
    _jointTargetInterpolationDurationSec =
        static_cast<double>(_robotCfg.controlCfg.decimation) *
        static_cast<double>(_robotCfg.controlCfg.dt);
    motion::Logger::instance().info(
        "Band-style joint target interpolation: enabled={}, duration={:.6f}s",
        _enableJointTargetInterpolation,
        _jointTargetInterpolationDurationSec);

    // waist joint kp/kd
    _robotCfg.controlCfg.waist_kp =
        config["LeggedRobotCfg"][_rlMode]["waist"]["kp"].as<float>();
    _robotCfg.controlCfg.waist_kd =
        config["LeggedRobotCfg"][_rlMode]["waist"]["kd"].as<float>();
    _waistKp = _robotCfg.controlCfg.waist_kp;
    _waistKd = _robotCfg.controlCfg.waist_kd;

    // get per-joint action scale from yaml
    _robotCfg.controlCfg.action_scale =
        config["LeggedRobotCfg"][_rlMode]["action_scale"]
            .as<std::map<std::string, float>>();

    // get normalization from yaml
    _robotCfg.clipObs =
        config["LeggedRobotCfg"][_rlMode]["normalization"]["clip_scales"]["clip_observations"]
            .as<double>();
    _robotCfg.clipActions =
        config["LeggedRobotCfg"][_rlMode]["normalization"]["clip_scales"]["clip_actions"]
            .as<double>();

    _cmd_vel_bias_x =
        config["LeggedRobotCfg"][_rlMode]["normalization"]["cmd_vel_bias"]["cmd_vel_bias_x"]
            .as<double>();
    _cmd_vel_bias_y =
        config["LeggedRobotCfg"][_rlMode]["normalization"]["cmd_vel_bias"]["cmd_vel_bias_y"]
            .as<double>();
    _cmd_vel_bias_yaw =
        config["LeggedRobotCfg"][_rlMode]["normalization"]["cmd_vel_bias"]["cmd_vel_bias_yaw"]
            .as<double>();

    // get size of model from yaml
    _observationSize =
        config["LeggedRobotCfg"][_rlMode]["size"]["observations_size"].as<int>();   // 45
    _actionsSize =
        config["LeggedRobotCfg"][_rlMode]["size"]["actions_size"].as<int>();       // 12
    _historySize =
        config["LeggedRobotCfg"][_rlMode]["size"]["history_size"].as<int>();       // 4
    _inputSize =
        config["LeggedRobotCfg"][_rlMode]["size"]["input_size"].as<int>();         // 180/184/196

    if (_actionsSize != _actuatedDofNum)
    {
      throw std::runtime_error("Configured actions_size does not match the 12 policy joints");
    }
    if (_observationSize != loco::kFrameSize || _historySize != loco::kHistorySize)
    {
      throw std::runtime_error("Loco sim2sim requires observations_size=45 and history_size=4");
    }
    const int historyElements = _observationSize * _historySize;
    if (_inputSize == historyElements)
      _gaitPhaseFrames = 0;
    else if (_inputSize == historyElements + loco::kPhaseSize)
      _gaitPhaseFrames = 1;
    else if (_inputSize == historyElements + loco::kPhaseSize * _historySize)
      _gaitPhaseFrames = _historySize;
    else
      throw std::runtime_error("Loco input_size must be 180, 184 or 196 (phase history)");
    _gaitPhaseTimeSec = 0.0;
    _gaitPhaseHistory.reset();
    _isfirstRecObs = true;
    motion::Logger::instance().info(
        "Gait phase: frames={}, period=1.0s, offsets=[0, 0.5], features={}",
        _gaitPhaseFrames, _gaitPhaseFrames * loco::kPhaseSize);

    const auto validateJointMapSize = [this](const auto &jointMap, const char *mapName)
    {
      if (jointMap.size() != static_cast<size_t>(_actuatedDofNum))
      {
        throw std::runtime_error(
            std::string("Config map '") + mapName + "' has " +
            std::to_string(jointMap.size()) + " entries; expected " +
            std::to_string(_actuatedDofNum));
      }
    };
    validateJointMapSize(_robotCfg.controlCfg.action_scale, "action_scale");
    validateJointMapSize(_robotCfg.controlCfg.stand_stiffness, "stand.stiffness");
    validateJointMapSize(_robotCfg.controlCfg.stand_damping, "stand.damping");
    validateJointMapSize(_robotCfg.controlCfg.walk_stiffness, "walk.stiffness");
    validateJointMapSize(_robotCfg.controlCfg.walk_damping, "walk.damping");

    // Resize vectors and matrices for the 22-action policy.
    _actions.resize(_actionsSize);           // 22
    _observations.resize(_inputSize);        // 180
    std::fill(_observations.begin(), _observations.end(), 0.0f);

    _lastActions.resize(_actionsSize);       // 22
    _lastActions.setZero();

    _command.x = 0;
    _command.y = 0;
    _command.yaw = 0;

    int actuatedDofNum = _actuatedDofNum;    // 22

    _zeroJointAngles.resize(actuatedDofNum);       // 22
    _standJointAngles.resize(actuatedDofNum);      // 22
    _defaultJointAngles.resize(actuatedDofNum);    // 22
    _actionScale.resize(actuatedDofNum);           // 22
    _defaultObsJointAngles.resize(12);             // 12 leg-only defaults for observation

    // Proprioception history is separate from the 4x4 gait-phase history.
    _proprioHistoryBuffer.resize(historyElements); // 45 * 4 = 180

    // Populate stand/default/zero joint angles and action scales in map-iteration order
    // std::map iterates alphabetically: leg_l1..l6, leg_r1..r6, upper_left1..5, upper_right1..5
    int index = 0;
    for (auto pair = _robotCfg.controlCfg.stand_joint_angle.begin();
         pair != _robotCfg.controlCfg.stand_joint_angle.end() && index < _actuatedDofNum;
         ++pair, ++index)
    {
      std::string partName = pair->first;
      _standJointAngles(index) = _robotCfg.controlCfg.stand_joint_angle[partName];
      _zeroJointAngles(index) = 0.0;
      _actions[index] = 0.0;
    }

    index = 0;
    for (auto pair = _robotCfg.controlCfg.default_joint_angle.begin();
         pair != _robotCfg.controlCfg.default_joint_angle.end() && index < _actuatedDofNum;
         ++pair, ++index)
    {
      std::string partName = pair->first;
      _defaultJointAngles(index) = _robotCfg.controlCfg.default_joint_angle[partName];
      motion::Logger::instance().info("default joint angle[{}]: {}", partName,
                                      _defaultJointAngles(index));
    }

    // Populate per-joint action scales
    index = 0;
    for (auto pair = _robotCfg.controlCfg.action_scale.begin();
         pair != _robotCfg.controlCfg.action_scale.end(); ++pair, ++index)
    {
      std::string partName = pair->first;
      _actionScale(index) = _robotCfg.controlCfg.action_scale[partName];
      motion::Logger::instance().info("action scale[{}]: {}", partName, _actionScale(index));
    }

    // Leg-only default joint angles for observation (first 12 of the 22 alphabetical order)
    // Those are leg_l1..l6 then leg_r1..r6
    _defaultObsJointAngles = _defaultJointAngles.head(12);

    // Initialize proprioception state (22-dof)
    _propri.jointPos.resize(_obsActuatorDofNum);
    _propri.jointVel.resize(_obsActuatorDofNum);
    _propri.baseAngVel.setZero();
    _propri.baseEulerXyz.setZero();

    motion::Logger::instance().info("New AMP config loaded: obs={}, actions={}, history={}, input={}",
                                    _observationSize, _actionsSize, _historySize, _inputSize);
  }

  double RLController::getDefaultJointAngle(const std::string &name) const
  {
    auto it = _robotCfg.controlCfg.default_joint_angle.find(name);
    if (it != _robotCfg.controlCfg.default_joint_angle.end())
    {
      return it->second;
    }
    return 0.0;
  }

  /**
   * @description: load model from onnx file
   */
  void RLController::loadModel(const std::string &model_path)
  {
    _onnxEnvPtr.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
    char tmp_last_thread_name[64] = {0x00};
    pthread_getname_np(pthread_self(), tmp_last_thread_name, sizeof(tmp_last_thread_name));
    pthread_setname_np(pthread_self(), std::string("hlorin_rlonnx").substr(0, 15).c_str());

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);
    sessionOptions.SetIntraOpNumThreads(5);
    sessionOptions.AddConfigEntry("session.intra_op_thread_affinities", "3,4;7,8;9,10;11,12");

    // Check for CUDA support
    {
      auto providers = Ort::GetAvailableProviders();
      auto cudaAvailable =
          std::find(providers.begin(), providers.end(), "CUDAExecutionProvider");
      OrtCUDAProviderOptions cudaOption;
      if (cudaAvailable == providers.end())
      {
        motion::Logger::instance().info(
            "GPU is not supported by your ONNXRuntime build. Fallback to CPU.");
        motion::Logger::instance().info("Inference device: CPU");
      }
      else if (cudaAvailable != providers.end())
      {
        motion::Logger::instance().info("Inference device: GPU");
        sessionOptions.AppendExecutionProvider_CUDA(cudaOption);
      }
      else
      {
        motion::Logger::instance().info("Inference device: CPU");
      }
    }

    _sessionPtr = std::make_unique<Ort::Session>(*_onnxEnvPtr, model_path.c_str(), sessionOptions);

    // get input and output info
    inputNames_.clear();
    outputNames_.clear();
    inputShapes_.clear();
    outputShapes_.clear();

    pthread_setname_np(pthread_self(),
                       std::string(tmp_last_thread_name).substr(0, 15).c_str());

    Ort::AllocatorWithDefaultOptions allocator;
    for (size_t i = 0; i < _sessionPtr->GetInputCount(); i++)
    {
      auto inputnamePtr = _sessionPtr->GetInputNameAllocated(i, allocator);
      inputNodeNameAllocatedStrings.push_back(std::move(inputnamePtr));
      inputNames_.push_back(inputNodeNameAllocatedStrings.back().get());
      inputShapes_.push_back(
          _sessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> shape =
          _sessionPtr->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      for (size_t j = 0; j < shape.size(); ++j)
      {
        motion::Logger::instance().info("inputShapes_[{}][{}] = {}", i, j, shape[j]);
      }
    }

    for (size_t i = 0; i < _sessionPtr->GetOutputCount(); i++)
    {
      auto outputnamePtr = _sessionPtr->GetOutputNameAllocated(i, allocator);
      outputNodeNameAllocatedStrings.push_back(std::move(outputnamePtr));
      outputNames_.push_back(outputNodeNameAllocatedStrings.back().get());
      motion::Logger::instance().info("output name: {}", outputNames_.back());
      outputShapes_.push_back(
          _sessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> shape =
          _sessionPtr->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      for (size_t j = 0; j < shape.size(); ++j)
      {
        motion::Logger::instance().info("outputShapes_[{}][{}] = {}", i, j, shape[j]);
      }
    }

    if (inputShapes_.size() != 1 || inputShapes_[0].size() != 2 ||
        (inputShapes_[0][1] != -1 && inputShapes_[0][1] != _inputSize))
    {
      throw std::runtime_error("ONNX input shape does not match configured input_size");
    }
    if (outputShapes_.size() != 1 || outputShapes_[0].size() != 2 ||
        (outputShapes_[0][1] != -1 && outputShapes_[0][1] != _actionsSize))
    {
      throw std::runtime_error("ONNX output shape does not match configured actions_size");
    }
  }

  /**
   * @description: Soft back mode — gradually reduce torque to zero
   */
  void RLController::handleSoftBackMode(double _stateTorquePercent,
                                         std::vector<float> &currentJointTor,
                                         double lastJointTor[],
                                         JointOutputData &jointOutputData)
  {
    int index = 0;
    for (auto pair = _robotCfg.controlCfg.walk_stiffness.begin();
         pair != _robotCfg.controlCfg.walk_stiffness.end(); ++pair, ++index)
    {
      jointOutputData.jointPosDes[index] = 0.0;
      jointOutputData.jointVelDes[index] = 0.0;
      jointOutputData.jointTorqueDes[index] =
          (1 - _stateTorquePercent) * lastJointTor[index];
      jointOutputData.kp[index] = 0.0;
      jointOutputData.kd[index] = 0.0;
    }
  }

  /**
   * @description: Zero mode — interpolate toward zero joint angles
   */
  void RLController::handleZeroMode(double standPercent,
                                     std::vector<float> &currentJointPos,
                                     JointOutputData &jointOutputData)
  {
    int index = 0;
    for (auto pair = _robotCfg.controlCfg.walk_stiffness.begin();
         pair != _robotCfg.controlCfg.walk_stiffness.end(); ++pair, ++index)
    {
      std::string partName = pair->first;
      jointOutputData.jointPosDes[index] =
          currentJointPos[index] * (1 - standPercent) + _zeroJointAngles[index] * standPercent;
      jointOutputData.jointVelDes[index] = 0.0;
      jointOutputData.jointTorqueDes[index] = 0.0;
      jointOutputData.kp[index] = _robotCfg.controlCfg.stand_stiffness[partName];
      jointOutputData.kd[index] = _robotCfg.controlCfg.stand_damping[partName];
    }
  }

  /**
   * @description: Publish one complete 1 kHz input frame for the policy thread.
   */
  void RLController::commitInputSnapshot()
  {
    std::lock_guard<std::mutex> lock(_inputSnapshotMutex);
    _pendingInputSnapshot = _rlInputData;
    _hasInputSnapshot = true;
  }

  /**
   * @description: Publish the latest complete policy output.
   * @return true after the first valid policy frame has been received.
   */
  bool RLController::handleWalkMode(JointOutputData &jointOutputData)
  {
    creatThreadFirst(jointOutputData.jointPosDes.size());
    const bool hasNewPolicyTarget = updateJointOutputData(_latestPolicyTarget);

    if (!_hasPolicyOutput)
    {
      handleEnd();
      return false;
    }

    if (!_enableJointTargetInterpolation)
    {
      jointOutputData = _latestPolicyTarget;
    }
    else
    {
      if (hasNewPolicyTarget)
      {
        beginJointTargetInterpolation(_latestPolicyTarget);
      }

      // Kp/Kd and all non-position fields follow the newest policy target,
      // matching Band's LegController. Only q_des is made continuous.
      jointOutputData = _latestPolicyTarget;
      evaluateJointTargetInterpolation(
          std::chrono::steady_clock::now(), jointOutputData.jointPosDes);
    }

    handleEnd();
    return true;
  }

  /**
   * @description Reset interpolation when entering RL control. The first
   *              policy segment starts at the measured joint position, which
   *              prevents a stale target or first-frame position step.
   */
  void RLController::resetJointTargetInterpolation(
      const std::vector<float> &currentJointPos)
  {
    // Restart the policy clock on RL-mode entry, not whenever the command stops.
    // The inference thread performs the reset to avoid racing its clock update.
    _resetGaitPhase.store(true, std::memory_order_release);
    _jointTargetInterpolationActive = false;
    _jointTargetInterpolationHasInitial =
        currentJointPos.size() >= static_cast<size_t>(_actuatedDofNum);

    _jointTargetInterpolationInitial.assign(_actuatedDofNum, 0.0f);
    if (_jointTargetInterpolationHasInitial)
    {
      std::copy_n(currentJointPos.begin(), _actuatedDofNum,
                  _jointTargetInterpolationInitial.begin());
    }

    // Do not reuse a policy frame produced in the previous robot mode.
    _hasPolicyOutput = false;
    _newDataAvailable.store(false, std::memory_order_release);
  }

  /**
   * @description Start one time-based segment from the currently commanded
   *              position to the newest policy target.
   */
  void RLController::beginJointTargetInterpolation(
      const JointOutputData &newTarget)
  {
    const auto now = std::chrono::steady_clock::now();
    const size_t targetSize = newTarget.jointPosDes.size();

    if (_jointTargetInterpolationStart.size() != targetSize)
    {
      _jointTargetInterpolationStart.assign(targetSize, 0.0f);
      _jointTargetInterpolationEnd.assign(targetSize, 0.0f);
      _jointTargetInterpolationActive = false;
    }

    if (_jointTargetInterpolationActive)
    {
      // Rebase at the position that is actually being commanded now. This is
      // continuous even if the 50 Hz inference thread has scheduling jitter.
      evaluateJointTargetInterpolation(now, _jointTargetInterpolationStart);
    }
    else if (_jointTargetInterpolationHasInitial &&
             _jointTargetInterpolationInitial.size() == targetSize)
    {
      _jointTargetInterpolationStart = _jointTargetInterpolationInitial;
    }
    else
    {
      // Same first-target behaviour as Band when no measured start is known.
      _jointTargetInterpolationStart = newTarget.jointPosDes;
    }

    _jointTargetInterpolationEnd = newTarget.jointPosDes;
    _jointTargetInterpolationStartTime = now;
    _jointTargetInterpolationActive = true;
    _jointTargetInterpolationHasInitial = false;
  }

  /**
   * @description Evaluate q(t) = q0 + alpha * (q1 - q0), with alpha based on
   *              steady-clock elapsed time rather than loop-count phase.
   */
  void RLController::evaluateJointTargetInterpolation(
      const std::chrono::steady_clock::time_point &now,
      std::vector<float> &position) const
  {
    if (!_jointTargetInterpolationActive ||
        _jointTargetInterpolationStart.size() !=
            _jointTargetInterpolationEnd.size())
    {
      return;
    }

    const double elapsedSec =
        std::chrono::duration<double>(
            now - _jointTargetInterpolationStartTime).count();
    const double alpha = std::clamp(
        elapsedSec / _jointTargetInterpolationDurationSec, 0.0, 1.0);

    position.resize(_jointTargetInterpolationEnd.size());
    for (size_t i = 0; i < position.size(); ++i)
    {
      position[i] = static_cast<float>(
          _jointTargetInterpolationStart[i] +
          alpha * (_jointTargetInterpolationEnd[i] -
                   _jointTargetInterpolationStart[i]));
    }
  }

  /**
   * @description: Post-step counter updates
   */
  void RLController::handleEnd()
  {
    stand_command = false;
    if (stand_command)
    {
      _loopCount = 0;
    }
    else
    {
      _loopCount++;
    }

    // Increment RL loop count when moving. Do not modify _command here: it is
    // owned by the 50 Hz policy thread and includes command slew limiting.
    if (_command.x > 0.1 || _command.x < -0.1)
    {
      _rlLoopCount++;
    }
  }

  /**
   * @description: Create RL thread on first call
   */
  void RLController::creatThreadFirst(size_t dim)
  {
    if (_rl_thread == nullptr)
    {
      _rl_thread = std::make_shared<std::thread>(&RLController::threadRunning, this, dim);
    }
  }

  /**
   * @description: Read the latest RL output from the double buffer
   */
  bool RLController::updateJointOutputData(JointOutputData &jointOutputData)
  {
    if (_newDataAvailable.load())
    {
      int readIndex = _readIndex.load();
      if (readIndex < 0 || readIndex >= static_cast<int>(_outputBuffers.size()))
      {
        motion::Logger::instance().error(
            "updateJointOutputData() invalid readIndex: {}, buffer size: {}",
            readIndex, _outputBuffers.size());
        return false;
      }
      jointOutputData = _outputBuffers[readIndex];
      _hasPolicyOutput = true;
      _newDataAvailable.store(false);
      _noDataCount = 0;
      return true;
    }

    if (++_noDataCount > 2000)
    {
      motion::Logger::instance().error(
          "updateJointOutputData() no new data for {} cycles", _noDataCount);
      _noDataCount = 0;
    }
    return false;
  }

  /**
   * @description: RL inference thread running at policy frequency (50Hz with decimation=20)
   */
  void RLController::threadRunning(size_t dim)
  {
    JointOutputData jointOutputData(dim);
    int _calcCount = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> _lastCalcTime =
        std::chrono::high_resolution_clock::now();
    pthread_setname_np(pthread_self(), std::string("hlorin_rlmain").substr(0, 15).c_str());

    // Use the same policy period for scheduling and the gait clock.
    const double policyDt = static_cast<double>(_robotCfg.controlCfg.dt) *
                            _robotCfg.controlCfg.decimation;
    rclcpp::Rate rate(1.0 / policyDt);

    while (!_stopThread.load())
    {
      // Save last actions before computing new ones
      for (int i = 0; i < _actionsSize; i++)
      {
        _lastActions(i, 0) = _actions[i];
      }

      // Run AMP inference
      if (_rlMode == "amp" || _rlMode == "amp_arm")
      {
        useRlAmpModel(jointOutputData);
      }

      // Diagnostic: log max action jump between consecutive inferences
      {
        float maxDelta = 0.0f;
        int maxIdx = 0;
        float sumDelta = 0.0f;
        for (int i = 0; i < _actionsSize; i++)
        {
          float delta = std::abs(_actions[i] - static_cast<float>(_lastActions(i, 0)));
          sumDelta += delta;
          if (delta > maxDelta) { maxDelta = delta; maxIdx = i; }
        }
        // motion::Logger::instance().info(
            // "RL action delta: max={:.4f} idx={}, mean={:.4f}",
            // maxDelta, maxIdx, sumDelta / _actionsSize);
      }

      // Double-buffer swap
      int currentWrite = _writeIndex.load();
      if (currentWrite < 0 || currentWrite >= static_cast<int>(_outputBuffers.size()))
      {
        motion::Logger::instance().error(
            "threadRunning() invalid writeIndex: {}, buffer size: {}",
            currentWrite, _outputBuffers.size());
        return;
      }
      _outputBuffers[currentWrite] = jointOutputData;

      int readIndex = _readIndex.load();
      if (readIndex < 0 || readIndex >= static_cast<int>(_outputBuffers.size()))
      {
        motion::Logger::instance().error(
            "threadRunning() invalid readIndex: {}, buffer size: {}",
            readIndex, _outputBuffers.size());
        return;
      }
      if (currentWrite == readIndex)
      {
        motion::Logger::instance().error(
            "threadRunning() writing to active read buffer: {}", currentWrite);
      }

      _writeIndex.store(readIndex);
      _readIndex.store(currentWrite);
      _newDataAvailable.store(true);

      // Frequency monitoring
      _calcCount++;
      auto now = std::chrono::high_resolution_clock::now();
      float calcMs =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastCalcTime).count();
      if (calcMs >= 1000)
      {
        motion::Logger::instance().info("RL thread running at {} Hz", _calcCount);
        _lastCalcTime = now;
        _calcCount = 0;
      }

      rate.sleep();
    }
  }

  /**
   * @description: Run AMP model: compute observation → inference → joint commands
   */
  void RLController::useRlAmpModel(JointOutputData &jointOutputData)
  {
    if (!updateRLData())
    {
      return;
    }
    if (_resetGaitPhase.exchange(false, std::memory_order_acq_rel))
    {
      _gaitPhaseTimeSec = 0.0;
      _gaitPhaseHistory.reset();
    }
    computeAmpObservation();
    std::vector<float> output = computeActions(_observations);
    _actions = output;

    computeJointDes(output, jointOutputData);
    // First inference uses t=0. Advance by the simulated policy step (20 ms),
    // including zero-command standing; do not use the 1 kHz movement counter.
    _gaitPhaseTimeSec += static_cast<double>(_robotCfg.controlCfg.dt) *
                         _robotCfg.controlCfg.decimation;
  }

  /**
   * @description: Update proprioception state from sensor data
   *               and the command used by both history and gait phase
   */
  bool RLController::updateRLData()
  {
    // Copy one complete frame and release the lock before observation
    // construction and ONNX inference.
    {
      std::lock_guard<std::mutex> lock(_inputSnapshotMutex);
      if (!_hasInputSnapshot)
      {
        return false;
      }
      _policyInputSnapshot = _pendingInputSnapshot;
    }

    const RLInPutData &input = _policyInputSnapshot;

    for (int i = 0; i < _obsActuatorDofNum; ++i)
    {
      _propri.jointPos(i) = input.jointInputData.jointPos[i];
      _propri.jointVel(i) = input.jointInputData.jointVel[i];
    }

    for (size_t i = 0; i < 3; ++i)
    {
      _propri.baseAngVel(i) = input.angVel[i];
      _propri.baseEulerXyz(i) = input.rpy[i];
    }
    _propri.projectedGravity =
        input.quat.inverse() * Eigen::Vector3d(0.0, 0.0, -1.0);

    const double targetCmdVelX = input.baseCmdVel[0] + _cmd_vel_bias_x;
    if (_enableCmdVelSlewX)
    {
      const double policyDt = static_cast<double>(_robotCfg.controlCfg.dt) *
                              static_cast<double>(_robotCfg.controlCfg.decimation);
      const double maxCmdVelStep = _cmdVelSlewRateX * policyDt;
      _rampedCmdVelX += std::clamp(targetCmdVelX - _rampedCmdVelX,
                                  -maxCmdVelStep, maxCmdVelStep);
    }
    else
    {
      _rampedCmdVelX = targetCmdVelX;
    }
    _command.x = _rampedCmdVelX;
    if (_enable_standing_posture_adjustment && _forwardCommand)
    {
      _command.x = 0.31;
    }
    _command.y = input.baseCmdVel[1] + _cmd_vel_bias_y;
    _command.yaw = input.baseCmdVel[2] + _cmd_vel_bias_yaw;

    return true;
  }

  /**
   * @description: 180-dim history plus current phase [180:184] or phase history [180:196]
   *
   *   Single frame layout (45 dims):
   *     [0:3]   base_ang_vel         (3)  — raw, no scaling
   *     [3:6]   projected_gravity    (3)  — quat.inverse() * (0,0,-1)
   *     [6:9]   command              (3)  — [cmd_vx, cmd_vy, cmd_yaw]
   *     [9:21]  joint_pos_rel        (12) — leg joints only, relative to default
   *     [21:33] joint_vel_rel        (12) — leg joints only
   *     [33:45] last_action          (12)
   */
  void RLController::computeAmpObservation()
  {
    Eigen::Matrix<float, Eigen::Dynamic, 1> singleObs(_observationSize); // 45

    // [0:3] base_ang_vel — raw angular velocity
    singleObs.segment<3>(0) = _propri.baseAngVel.cast<float>();

    // [3:6] projected_gravity = quat⁻¹ * (0, 0, -1)
    // This transforms the world-frame gravity vector into the body frame
    singleObs.segment<3>(3) = _propri.projectedGravity.cast<float>();

    // [6:9] command: forward velocity, lateral velocity, yaw rate
    singleObs(6) = static_cast<float>(_command.x.load());
    singleObs(7) = static_cast<float>(_command.y.load());
    singleObs(8) = static_cast<float>(_command.yaw.load());

    // [9:21]  joint_pos_rel: 12 leg joints, relative to default position
    // [21:33] joint_vel_rel: 12 leg joint velocities
    for (int i = 0; i < 12; i++)
    {
      singleObs(9 + i) =
          static_cast<float>(_propri.jointPos(i) - _defaultObsJointAngles(i));
      singleObs(21 + i) = static_cast<float>(_propri.jointVel(i));
    }

    // [33:45] last_action: previous policy output (12 legs)
    for (int i = 0; i < 12; i++)
    {
      singleObs(33 + i) = static_cast<float>(_lastActions(i));
    }

    // --- History buffer: 4-step sliding window ---
    if (_isfirstRecObs)
    {
      // Initialize entire history with the first observation frame
      for (int i = 0; i < _historySize; i++)
      {
        _proprioHistoryBuffer.segment(i * _observationSize, _observationSize) = singleObs;
      }
      _isfirstRecObs = false;
    }

    // Slide buffer left, append new frame at the end
    const int historyElements = _observationSize * _historySize;
    _proprioHistoryBuffer.head(historyElements - _observationSize) =
        _proprioHistoryBuffer.tail(historyElements - _observationSize).eval();
    _proprioHistoryBuffer.tail(_observationSize) = singleObs;

    const auto phase = loco::gaitPhaseFeatures(
        _gaitPhaseTimeSec, _command.x.load(), _command.y.load(), _command.yaw.load());
    const float *phaseData = nullptr;
    if (_gaitPhaseFrames == 1)
      phaseData = phase.data();
    else if (_gaitPhaseFrames == _historySize)
      phaseData = _gaitPhaseHistory.append(phase);
    loco::packObservation(_proprioHistoryBuffer.data(), _historySize,
                          phaseData, _gaitPhaseFrames, _observations);

    // Clip observations
    float obsMin = static_cast<float>(-_robotCfg.clipObs);
    float obsMax = static_cast<float>(_robotCfg.clipObs);
    std::transform(_observations.begin(), _observations.end(), _observations.begin(),
                   [obsMin, obsMax](float x)
                   { return std::max(obsMin, std::min(obsMax, x)); });
  }

  /**
   * @description: Run ONNX inference
   * @param {vector<float>} &observations — 180/184/196-dim flattened observation
   * @return {vector<float>} 12-dim actions
   */
  std::vector<float> RLController::computeActions(const std::vector<float> &observations)
  {
    // Create input tensor from observations
    // Use concrete shape [1, inputSize] — ONNX model has dynamic batch dim which returns -1
    std::vector<Ort::Value> inputValues;
    std::vector<int64_t> concreteInputShape = {1, static_cast<int64_t>(_observations.size())};
    inputValues.push_back(
        Ort::Value::CreateTensor<float>(_memoryInfo,
                                        const_cast<float *>(_observations.data()),
                                        _observations.size(),
                                        concreteInputShape.data(),
                                        concreteInputShape.size()));

    // Run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues =
        _sessionPtr->Run(runOptions, inputNames_.data(), inputValues.data(), 1,
                         outputNames_.data(), 1);

    const size_t outputElementCount =
        outputValues[0].GetTensorTypeAndShapeInfo().GetElementCount();
    if (outputElementCount != static_cast<size_t>(_actionsSize))
    {
      throw std::runtime_error(
          "ONNX output element count does not match configured actions_size");
    }

    // Extract 22 actions
    for (int i = 0; i < _actionsSize; i++)
    {
      _actions[i] = *(outputValues[0].GetTensorMutableData<float>() + i);
    }

    // Clip actions
    double actionMin = -_robotCfg.clipActions;
    double actionMax = _robotCfg.clipActions;
    std::transform(_actions.begin(), _actions.end(), _actions.begin(),
                   [actionMin, actionMax](double x)
                   { return std::max(actionMin, std::min(actionMax, x)); });

    // Filter exactly once per policy update. Both vectors are owned by the
    // inference thread, so the 1 kHz control thread never observes a partial
    // action update. Alpha matches the working HLMotion amp_arm deployment.
    if (_isActionsFilter)
    {
      constexpr float kActionFilterAlpha = 0.7f;
      for (int i = 0; i < _actionsSize; ++i)
      {
        _actions[i] =
            (1.0f - kActionFilterAlpha) * static_cast<float>(_lastActions[i]) +
            kActionFilterAlpha * _actions[i];
      }
    }

    return _actions;
  }

  /**
   * @description: Convert 22-dim actions to joint position targets with per-joint scale
   *               Joint order (alphabetical map iteration):
   *                 [0:5]  leg_l1..l6,  [6:11] leg_r1..r6,
   *                 [12:16] upper_left1..5, [17:21] upper_right1..5
   */
  void RLController::computeJointDes(const std::vector<float> &actions,
                                      JointOutputData &jointOutputData)
  {
    int index = 0;
    for (auto pair = _robotCfg.controlCfg.walk_stiffness.begin();
         pair != _robotCfg.controlCfg.walk_stiffness.end(); ++pair, ++index)
    {
      std::string partName = pair->first;
      jointOutputData.jointPosDes[index] =
          actions[index] * _actionScale(index) + _defaultJointAngles(index);
      jointOutputData.jointVelDes[index] = 0.0;
      jointOutputData.jointTorqueDes[index] = 0.0;
      jointOutputData.kp[index] = pair->second;
      jointOutputData.kd[index] = _robotCfg.controlCfg.walk_damping[partName];
    }
  }

  /**
   * @description: Adjust standing posture when robot is static
   *               (leg joint indices unchanged: hips=0,6; knees=3,9)
   */
  void RLController::adjustStandingPosture(std::vector<float> &actions)
  {
    double cmd_x = _command.x.load();
    double cmd_yaw = _command.yaw.load();

    // Detect transition from moving to static
    if ((std::abs(cmd_x) > 0.1 || std::abs(cmd_yaw) > 0.1) && !_changingPosture)
    {
      _staticCounter = 0;
      _isStatic = false;
      _changingPosture = false;
    }

    // When static for long enough, trigger posture adjustment
    if (std::abs(cmd_x) <= 0.1 && std::abs(cmd_yaw) <= 0.1 && !_isStatic)
    {
      if (++_staticCounter > _standing_posture_static_threshold)
      {
        float leftHipPos = static_cast<float>(_propri.jointPos[0]);
        float rightHipPos = static_cast<float>(_propri.jointPos[6]);
        hipDiff = std::abs(leftHipPos - rightHipPos);

        if (hipDiff > (_standing_posture_tolerance * kDegToRad))
        {
          _targetActionHip = std::min(actions[0], actions[6]);
          _targetActionKnee = std::min(actions[3], actions[9]);
          _targetPosHip = std::min(leftHipPos, rightHipPos);
          _adjustLeftLeg = !(leftHipPos < rightHipPos);
          _isStatic = true;
          _changingPosture = true;
          _forwardCommand = true;
          _inPosition = true;
          _postAdjustCounter = 0;
        }
      }
    }

    if (_changingPosture)
    {
      int target_hip_idx = _adjustLeftLeg ? 0 : 6;
      int other_hip_idx = _adjustLeftLeg ? 6 : 0;
      int other_knee_idx = _adjustLeftLeg ? 9 : 3;

      if (_propri.jointPos[target_hip_idx] <
              _targetPosHip - 0.03 &&
          _inPosition)
      {
        _forwardCommand = false;
        _inPosition = false;
        actions[target_hip_idx] =
            std::clamp(actions[target_hip_idx], _targetActionHip, _targetActionHip + 0.015f);
        actions[other_hip_idx] = std::max(actions[other_hip_idx], _targetActionHip);
        actions[other_knee_idx] = std::max(actions[other_knee_idx], _targetActionKnee);
      }
      else if (!_inPosition)
      {
        if (++_postAdjustCounter > 10)
        {
          _changingPosture = false;
          _postAdjustCounter = 0;
        }
        actions[target_hip_idx] =
            std::clamp(actions[target_hip_idx], _targetActionHip, _targetActionHip + 0.015f);
        actions[other_hip_idx] = std::max(actions[other_hip_idx], _targetActionHip);
        actions[other_knee_idx] = std::max(actions[other_knee_idx], _targetActionKnee);
      }
      else
      {
        actions[other_hip_idx] = _targetActionHip;
        actions[other_knee_idx] = _targetActionKnee;
      }
    }

    // Final clip
    double actionMin = -_robotCfg.clipActions;
    double actionMax = _robotCfg.clipActions;
    std::transform(actions.begin(), actions.end(), actions.begin(),
                   [actionMin, actionMax](double x)
                   { return std::max(actionMin, std::min(actionMax, x)); });
  }

} // namespace rl
