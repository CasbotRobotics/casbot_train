#pragma once

#include <string>
#include <vector>

namespace robot
{

// 总关节数
constexpr int gTotalJointNum = 49;

// 腿部关节
constexpr int gLegJointNum = 12;       // 双腿总关节数
constexpr int gLegDof = 6;            // 单腿自由度
constexpr int gLegStartId = 0;         // 腿部起始ID
constexpr int gLegEndId = gLegStartId + gLegJointNum - 1; // 腿部结束ID
constexpr int gLeftLegStartId = 0;    // 左腿起始ID
constexpr int gRightLegStartId = 6;   // 右腿起始ID

// 机械臂关节
constexpr int gArmJointNum = 14;       // 双臂总关节数
constexpr int gArmDof = 7;            // 单臂自由度
constexpr int gArmStartId = 12;        // 机械臂起始ID
constexpr int gArmEndId = gArmStartId + gArmJointNum - 1; // 机械臂结束ID
constexpr int gLeftArmStartId = 12;   // 左臂起始ID (12-18)
constexpr int gRightArmStartId = 19;  // 右臂起始ID (19-25)

// 头部关节
constexpr int gHeadJointNum = 2;      // 头部总关节数
constexpr int gHeadDof = 2;             // 头部自由度
constexpr int gHeadStartId = 26;       // 头部起始ID (26-27)
constexpr int gHeadEndId = gHeadStartId + gHeadJointNum - 1; // 头部结束ID

// 腰部关节
constexpr int gWaistJointNum = 1;    // 腰部总关节数
constexpr int gWaistDof = 1;            // 腰部自由度
constexpr int gWaistStartId = 28;      // 腰部起始ID
constexpr int gWaistEndId = gWaistStartId + gWaistJointNum - 1; // 腰部结束ID

// 灵巧手关节
constexpr int gHandJointNum = 20;      // 双手总关节数
constexpr int gHandDof = 10;            // 单手自由度
constexpr int gHandStartId = 29;       // 灵巧手起始ID
constexpr int gHandEndId = gHandStartId + gHandJointNum - 1; // 灵巧手结束ID
constexpr int gLeftHandStartId = 29;  // 左手起始ID (29-38)
constexpr int gRightHandStartId = 39; // 右手起始ID (39-48)

inline bool isLegJoint(int joint_id) { return joint_id >= gLegStartId && joint_id <= gLegEndId; }
inline bool isArmJoint(int joint_id) { return joint_id >= gArmStartId && joint_id <= gArmEndId; }
inline bool isHeadJoint(int joint_id) { return joint_id >= gHeadStartId && joint_id <= gHeadEndId; }
inline bool isWaistJoint(int joint_id) { return joint_id >= gWaistStartId && joint_id <= gWaistEndId; }
inline bool isHandJoint(int joint_id) { return joint_id >= gHandStartId && joint_id <= gHandEndId; }

std::string resolveJointName(std::string joint_name);

std::vector<std::string> getLegJointNames();
std::vector<std::string> getArmJointNames();
std::vector<std::string> getHeadJointNames();
std::vector<std::string> getWaistJointNames();
std::vector<std::string> getHandJointNames();
std::vector<std::string> getAllJointNames();

int getJointId(std::string joint_name);
std::string getJointNameById(int joint_id);

}