#pragma once

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace rl
{
    struct RLRobotCfg
    {
        struct ControlCfg
        {
            std::map<std::string, double> stand_joint_angle;
            std::map<std::string, double> default_joint_angle;
            std::map<std::string, float> stand_stiffness;
            std::map<std::string, float> stand_damping;
            std::map<std::string, float> walk_stiffness;
            std::map<std::string, float> walk_damping;
            std::map<std::string, float> action_scale;
            float actionScale;
            int decimation;
            float user_torque_limit;
            float user_power_limit;
            float cycle_time;
            float dt;
            float waist_kp;
            float waist_kd;
            int start_idx;
            int end_idx;
            int load_length;
            int start_idx_ap;
            int end_idx_ap;
            int load_length_ap;
        };

        struct ObsScales
        {
            double linVel;
            double angVel;
            double dofPos;
            double dofVel;
            double quat;
            double heightMeasurements;
        };
        struct CmdVelBias
        {
            /* data */
            double cmd_vel_bias_x;
            double cmd_vel_bias_y;
            double cmd_vel_bias_yaw;
        };

        double clipActions;
        double clipObs;
        ObsScales obsScales;
        ControlCfg controlCfg;
        CmdVelBias cmdVelBias;
    };

    struct Proprioception
    {
        Eigen::Matrix<double, Eigen::Dynamic, 1> jointPos;
        Eigen::Matrix<double, Eigen::Dynamic, 1> jointVel;
        Eigen::Matrix<double, 3, 1> baseAngVel;
        Eigen::Matrix<double, 3, 1> baseEulerXyz;
        Eigen::Matrix<double, 3, 1> projectedGravity;
    };

    struct Command
    {
        std::atomic<double> x;
        std::atomic<double> y;
        std::atomic<double> yaw;
        std::atomic<double> height;
    };

    struct JointInputData
    {
        std::vector<float> jointPos;
        std::vector<float> jointVel;
        std::vector<float> jointTorque;
        JointInputData(int dim) : jointPos(dim, 0.0f),
                                  jointVel(dim, 0.0f),
                                  jointTorque(dim, 0.0f)
        {
        }
    };

    class RLInPutData
    {
    public:
        JointInputData jointInputData;
        Eigen::Quaterniond quat = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
        Eigen::Vector3d rpy = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d angVel = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 1> baseCmdVel = Eigen::Matrix<double, 3, 1>::Zero(); // x, y, yaw
        RLInPutData() = default;
        RLInPutData(int legDOF, int armDOF) : jointInputData(legDOF + armDOF)
        {
        }
    };

    struct JointOutputData
    {
        std::vector<float> jointPosDes;
        std::vector<float> jointVelDes;
        std::vector<float> jointTorqueDes;
        std::vector<float> kp;
        std::vector<float> kd;

        JointOutputData() = default;

        JointOutputData(int dim) : jointPosDes(dim, 0.0f),
                                   jointVelDes(dim, 0.0f),
                                   jointTorqueDes(dim, 0.0f),
                                   kp(dim, 0.0f),
                                   kd(dim, 0.0f)
        {
        }
    };

    class RLOutputData
    {
    public:
        JointOutputData jointOutputData;

        RLOutputData() = default;

        RLOutputData(int legDOF, int armDOF) : jointOutputData(legDOF + armDOF)
        {
        }
    };

} // namespace rl
