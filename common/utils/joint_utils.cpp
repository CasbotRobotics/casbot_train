#include "joint_utils.hpp"
#include <joint_info/joint_info.hpp>
namespace robot
{
bool getJointDataFromOutputStruct(const message::outputStruct &outputStruct, sensor_msgs::msg::JointState& joint_data)
{
    joint_data.name.resize(robot::gTotalJointNum);
    joint_data.position.resize(robot::gTotalJointNum);
    joint_data.velocity.resize(robot::gTotalJointNum);
    joint_data.effort.resize(robot::gTotalJointNum);
    for (size_t i = 0; i < robot::gLegJointNum; ++i)
    {
        joint_data.name[robot::gLegStartId + i] = robot::getJointNameById(robot::gLegStartId + i);
        joint_data.position[robot::gLegStartId + i] = outputStruct.leg_cmd[i].pos_des;
        joint_data.velocity[robot::gLegStartId + i] = outputStruct.leg_cmd[i].vel_des;
        joint_data.effort[robot::gLegStartId + i] = 0.0;
    }
    for (size_t i = 0; i < robot::gArmJointNum; ++i)
    {
        joint_data.name[robot::gArmStartId + i] = robot::getJointNameById(robot::gArmStartId + i);
        joint_data.position[robot::gArmStartId + i] = outputStruct.arm_cmd[i].pos_des;
        joint_data.velocity[robot::gArmStartId + i] = outputStruct.arm_cmd[i].vel_des;
        joint_data.effort[robot::gArmStartId + i] = 0.0;
    }
    for (size_t i = 0; i < robot::gHeadJointNum; ++i)
    {
        joint_data.name[robot::gHeadStartId + i] = robot::getJointNameById(robot::gHeadStartId + i);
        joint_data.position[robot::gHeadStartId + i] = outputStruct.head_cmd[i].pos_des;
        joint_data.velocity[robot::gHeadStartId + i] = outputStruct.head_cmd[i].vel_des;
        joint_data.effort[robot::gHeadStartId + i] = 0.0;
    }
    joint_data.name[robot::gWaistStartId] = robot::getJointNameById(robot::gWaistStartId);
    joint_data.position[robot::gWaistStartId] = outputStruct.waist_cmd[0].pos_des;
    joint_data.velocity[robot::gWaistStartId] = outputStruct.waist_cmd[0].vel_des;
    joint_data.effort[robot::gWaistStartId] = 0.0;
    for (size_t i = 0; i < robot::gHandJointNum; ++i)
    {
        joint_data.name[robot::gHandStartId + i] = robot::getJointNameById(robot::gHandStartId + i);
        joint_data.position[robot::gHandStartId + i] = outputStruct.hand_cmd[i].pos_des;
        joint_data.velocity[robot::gHandStartId + i] = outputStruct.hand_cmd[i].vel_des;
        joint_data.effort[robot::gHandStartId + i] = 0.0;
    }
    return true;
}

bool getPdGainsFromOutputStruct(const message::outputStruct &outputStruct,
                                std::vector<double> &kp,
                                std::vector<double> &kd)
{
    kp.assign(robot::gTotalJointNum, 0.0);
    kd.assign(robot::gTotalJointNum, 0.0);

    for (size_t i = 0; i < robot::gLegJointNum; ++i)
    {
        kp[robot::gLegStartId + i] = outputStruct.leg_cmd[i].kp;
        kd[robot::gLegStartId + i] = outputStruct.leg_cmd[i].kd;
    }
    for (size_t i = 0; i < robot::gArmJointNum; ++i)
    {
        kp[robot::gArmStartId + i] = outputStruct.arm_cmd[i].kp;
        kd[robot::gArmStartId + i] = outputStruct.arm_cmd[i].kd;
    }
    for (size_t i = 0; i < robot::gHeadJointNum; ++i)
    {
        kp[robot::gHeadStartId + i] = outputStruct.head_cmd[i].kp;
        kd[robot::gHeadStartId + i] = outputStruct.head_cmd[i].kd;
    }
    kp[robot::gWaistStartId] = outputStruct.waist_cmd[0].kp;
    kd[robot::gWaistStartId] = outputStruct.waist_cmd[0].kd;
    for (size_t i = 0; i < robot::gHandJointNum; ++i)
    {
        kp[robot::gHandStartId + i] = outputStruct.hand_cmd[i].kp;
        kd[robot::gHandStartId + i] = outputStruct.hand_cmd[i].kd;
    }
    return true;
}

bool setJointDataInInputStruct(const sensor_msgs::msg::JointState &joint_state, message::inputStruct &inputStruct)
{
    if (joint_state.name.size() != joint_state.position.size())
    {
        return false;
    }
    for (size_t i = 0; i < joint_state.name.size(); ++i)
    {
        const int joint_id = robot::getJointId(joint_state.name[i]);
        const double pos = joint_state.position[i];
        const double vel = (i < joint_state.velocity.size()) ? joint_state.velocity[i] : 0.0;
        const double torq = (i < joint_state.effort.size()) ? joint_state.effort[i] : 0.0;

        if (joint_id >= robot::gLegStartId && joint_id <= robot::gLegEndId)
        {
            const int leg_index = joint_id - robot::gLegStartId;
            inputStruct.leg_data[leg_index].pos_real = pos;
            inputStruct.leg_data[leg_index].vel_real = vel;
            inputStruct.leg_data[leg_index].torq_real = torq;
        }
        else if (joint_id >= robot::gArmStartId && joint_id <= robot::gArmEndId)
        {
            const int arm_index = joint_id - robot::gArmStartId;
            inputStruct.arm_data[arm_index].pos_real = pos;
            inputStruct.arm_data[arm_index].vel_real = vel;
            inputStruct.arm_data[arm_index].torq_real = torq;
        }
        else if (joint_id >= robot::gHeadStartId && joint_id <= robot::gHeadEndId)
        {
            const int head_index = joint_id - robot::gHeadStartId;
            inputStruct.head_data[head_index].pos_real = pos;
            inputStruct.head_data[head_index].vel_real = vel;
            inputStruct.head_data[head_index].torq_real = torq;
        }
        else if (joint_id == robot::gWaistStartId)
        {
            inputStruct.waist_data[0].pos_real = pos;
            inputStruct.waist_data[0].vel_real = vel;
            inputStruct.waist_data[0].torq_real = torq;
        }
        else if (joint_id >= robot::gHandStartId && joint_id <= robot::gHandEndId)
        {
            const int hand_index = joint_id - robot::gHandStartId;
            inputStruct.hand_data[hand_index].pos_real = pos;
            inputStruct.hand_data[hand_index].vel_real = vel;
            inputStruct.hand_data[hand_index].torq_real = torq;
        }
    }
    return true;
}

bool setImuDataInInputStruct(const sensor_msgs::msg::Imu &imu_msg, message::inputStruct &inputStruct)
{
    inputStruct.orientation[0] = imu_msg.orientation.w;
    inputStruct.orientation[1] = imu_msg.orientation.x;
    inputStruct.orientation[2] = imu_msg.orientation.y;
    inputStruct.orientation[3] = imu_msg.orientation.z;
    inputStruct.gyr[0] = imu_msg.angular_velocity.x;
    inputStruct.gyr[1] = imu_msg.angular_velocity.y;
    inputStruct.gyr[2] = imu_msg.angular_velocity.z;
    inputStruct.acc[0] = imu_msg.linear_acceleration.x;
    inputStruct.acc[1] = imu_msg.linear_acceleration.y;
    inputStruct.acc[2] = imu_msg.linear_acceleration.z;
    inputStruct.rpy[0] = imu_msg.orientation_covariance[0];
    inputStruct.rpy[1] = imu_msg.orientation_covariance[1];
    inputStruct.rpy[2] = imu_msg.orientation_covariance[2];
    return true;
}
} // namespace robot
