#include "joint_info/joint_info.hpp"

#include <array>
#include <unordered_map>
#include "joint_info.hpp"

namespace robot
{
static const std::array<std::string, gTotalJointNum> kCanonicalNames = {
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint",
    "leg_l5_joint", "leg_l6_joint", "leg_r1_joint", "leg_r2_joint",
    "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_pitch_joint", "left_wrist_yaw_joint",
    "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_pitch_joint", "right_wrist_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint", "waist_yaw_joint",
    "left_thumb_metacarpal_joint", "left_thumb_proximal_joint",
    "left_index_proximal_joint", "left_index_distal_joint",
    "left_middle_proximal_joint", "left_middle_distal_joint",
    "left_ring_proximal_joint", "left_ring_distal_joint",
    "left_pinky_proximal_joint", "left_pinky_distal_joint",
    "right_thumb_metacarpal_joint", "right_thumb_proximal_joint",
    "right_index_proximal_joint", "right_index_distal_joint",
    "right_middle_proximal_joint", "right_middle_distal_joint",
    "right_ring_proximal_joint", "right_ring_distal_joint",
    "right_pinky_proximal_joint", "right_pinky_distal_joint",
};

static const std::unordered_map<std::string, int> gJointNameToId = {
    // 腿
    {"leg_l1_joint", 0},
    {"leg_l2_joint", 1},
    {"leg_l3_joint", 2},
    {"leg_l4_joint", 3},
    {"leg_l5_joint", 4},
    {"leg_l6_joint", 5},   
    {"leg_r1_joint", 6},
    {"leg_r2_joint", 7}, 
    {"leg_r3_joint", 8},  
    {"leg_r4_joint", 9},  
    {"leg_r5_joint", 10},
    {"leg_r6_joint", 11},
    // 臂
    {"left_shoulder_pitch_joint", 12},
    {"left_shoulder_roll_joint", 13},
    {"left_shoulder_yaw_joint", 14},
    {"left_elbow_pitch_joint", 15},
    {"left_wrist_yaw_joint", 16},
    {"left_wrist_pitch_joint", 17},
    {"left_wrist_roll_joint", 18},
    {"right_shoulder_pitch_joint", 19},
    {"right_shoulder_roll_joint", 20},
    {"right_shoulder_yaw_joint", 21},
    {"right_elbow_pitch_joint", 22},
    {"right_wrist_yaw_joint", 23},
    {"right_wrist_pitch_joint", 24},
    {"right_wrist_roll_joint", 25},
    
    // 头
    {"head_yaw_joint", 26},
    {"head_pitch_joint", 27},
    // 腰
    {"waist_yaw_joint", 28},

    // 拇指根关节,拇指近节关节,
    // 食指近节关节,食指远节关节,
    // 中指近节关节,中指远节关节,
    // 无名指近节关节,无名指远节关节,
    // 小指近节关节,小指远节关节

    {"left_thumb_metacarpal_joint", 29},
    {"left_thumb_proximal_joint", 30},
    {"left_index_proximal_joint", 31},
    {"left_index_distal_joint", 32},
    {"left_middle_proximal_joint", 33},
    {"left_middle_distal_joint", 34},
    {"left_ring_proximal_joint", 35},
    {"left_ring_distal_joint", 36},
    {"left_pinky_proximal_joint", 37},
    {"left_pinky_distal_joint", 38},

    {"right_thumb_metacarpal_joint", 39},
    {"right_thumb_proximal_joint", 40},
    {"right_index_proximal_joint", 41},
    {"right_index_distal_joint", 42},
    {"right_middle_proximal_joint", 43},
    {"right_middle_distal_joint", 44},
    {"right_ring_proximal_joint", 45},
    {"right_ring_distal_joint", 46},
    {"right_pinky_proximal_joint", 47},
    {"right_pinky_distal_joint", 48},
};

static const std::unordered_map<std::string, std::string> joint_name_map = {
    // 左腿
    {"leg_l1_joint",                    "leg_l1_joint"},
    {"left_leg_pelvic_pitch_joint",     "leg_l1_joint"},
    {"left_leg_pelvic_pitch",           "leg_l1_joint"},
    {"LJ0",                             "leg_l1_joint"},

    {"leg_l2_joint",                    "leg_l2_joint"},
    {"left_leg_pelvic_roll_joint",      "leg_l2_joint"},
    {"left_leg_pelvic_roll",            "leg_l2_joint"},
    {"LJ1",                             "leg_l2_joint"},

    {"leg_l3_joint",                    "leg_l3_joint"},
    {"left_leg_pelvic_yaw_joint",       "leg_l3_joint"},
    {"left_leg_pelvic_yaw",             "leg_l3_joint"},
    {"LJ2",                             "leg_l3_joint"},

    {"leg_l4_joint",                    "leg_l4_joint"},
    {"left_leg_knee_pitch_joint",       "leg_l4_joint"},
    {"left_leg_knee_pitch",             "leg_l4_joint"},
    {"LJ3",                             "leg_l4_joint"},

    {"leg_l5_joint",                    "leg_l5_joint"},
    {"left_leg_ankle_pitch_joint",      "leg_l5_joint"},
    {"left_leg_ankle_pitch",            "leg_l5_joint"},
    {"LJ4",                             "leg_l5_joint"},

    {"leg_l6_joint",                    "leg_l6_joint"},
    {"left_leg_ankle_roll_joint",       "leg_l6_joint"},
    {"left_leg_ankle_roll",             "leg_l6_joint"},
    {"LJ5",                             "leg_l6_joint"},

    // 右腿
    {"leg_r1_joint",                    "leg_r1_joint"},
    {"right_leg_pelvic_pitch_joint",    "leg_r1_joint"},
    {"right_leg_pelvic_pitch",          "leg_r1_joint"},
    {"RJ6",                             "leg_r1_joint"},

    {"leg_r2_joint",                    "leg_r2_joint"},
    {"right_leg_pelvic_roll_joint",     "leg_r2_joint"},
    {"right_leg_pelvic_roll",           "leg_r2_joint"},
    {"RJ7",                             "leg_r2_joint"},

    {"leg_r3_joint",                    "leg_r3_joint"},
    {"right_leg_pelvic_yaw_joint",      "leg_r3_joint"},
    {"right_leg_pelvic_yaw",            "leg_r3_joint"},
    {"RJ8",                             "leg_r3_joint"},

    {"leg_r4_joint",                    "leg_r4_joint"},
    {"right_leg_knee_pitch_joint",      "leg_r4_joint"},
    {"right_leg_knee_pitch",            "leg_r4_joint"},
    {"RJ9",                             "leg_r4_joint"},

    {"leg_r5_joint",                    "leg_r5_joint"},
    {"right_leg_ankle_pitch_joint",     "leg_r5_joint"},
    {"right_leg_ankle_pitch",           "leg_r5_joint"},
    {"RJ10",                            "leg_r5_joint"},

    {"leg_r6_joint",                    "leg_r6_joint"},
    {"right_leg_ankle_roll_joint",      "leg_r6_joint"},
    {"right_leg_ankle_roll",            "leg_r6_joint"},
    {"RJ11",                            "leg_r6_joint"},

    // 左臂
    {"arm_l1_joint",                    "left_shoulder_pitch_joint"},
    {"left_shoulder_pitch_joint",       "left_shoulder_pitch_joint"},
    {"left_shoulder_pitch",             "left_shoulder_pitch_joint"},

    {"arm_l2_joint",                    "left_shoulder_roll_joint"},
    {"left_shoulder_roll_joint",        "left_shoulder_roll_joint"},
    {"left_shoulder_roll",              "left_shoulder_roll_joint"},

    {"arm_l3_joint",                    "left_shoulder_yaw_joint"},
    {"left_shoulder_yaw_joint",         "left_shoulder_yaw_joint"},
    {"left_shoulder_yaw",               "left_shoulder_yaw_joint"},

    {"arm_l4_joint",                    "left_elbow_pitch_joint"},
    {"left_elbow_pitch_joint",          "left_elbow_pitch_joint"},
    {"left_elbow_pitch",                "left_elbow_pitch_joint"},

    {"arm_l5_joint",                    "left_wrist_yaw_joint"},
    {"left_wrist_yaw_joint",            "left_wrist_yaw_joint"},
    {"left_wrist_yaw",                  "left_wrist_yaw_joint"},

    {"arm_l6_joint",                    "left_wrist_pitch_joint"},
    {"left_wrist_pitch_joint",          "left_wrist_pitch_joint"},
    {"left_wrist_pitch",                "left_wrist_pitch_joint"},

    {"arm_l7_joint",                    "left_wrist_roll_joint"},
    {"left_wrist_roll_joint",           "left_wrist_roll_joint"},
    {"left_wrist_roll",                 "left_wrist_roll_joint"},

    // 右臂
    {"arm_r1_joint",                    "right_shoulder_pitch_joint"},
    {"right_shoulder_pitch_joint",      "right_shoulder_pitch_joint"},
    {"right_shoulder_pitch",            "right_shoulder_pitch_joint"},

    {"arm_r2_joint",                    "right_shoulder_roll_joint"},
    {"right_shoulder_roll_joint",       "right_shoulder_roll_joint"},
    {"right_shoulder_roll",             "right_shoulder_roll_joint"},

    {"arm_r3_joint",                    "right_shoulder_yaw_joint"},
    {"right_shoulder_yaw_joint",        "right_shoulder_yaw_joint"},
    {"right_shoulder_yaw",              "right_shoulder_yaw_joint"},

    {"arm_r4_joint",                    "right_elbow_pitch_joint"},
    {"right_elbow_pitch_joint",         "right_elbow_pitch_joint"},
    {"right_elbow_pitch",               "right_elbow_pitch_joint"},

    {"arm_r5_joint",                    "right_wrist_yaw_joint"},
    {"right_wrist_yaw_joint",           "right_wrist_yaw_joint"},
    {"right_wrist_yaw",                 "right_wrist_yaw_joint"},

    {"arm_r6_joint",                    "right_wrist_pitch_joint"},
    {"right_wrist_pitch_joint",         "right_wrist_pitch_joint"},
    {"right_wrist_pitch",               "right_wrist_pitch_joint"},

    {"arm_r7_joint",                    "right_wrist_roll_joint"},
    {"right_wrist_roll_joint",          "right_wrist_roll_joint"},
    {"right_wrist_roll",                "right_wrist_roll_joint"},

    // 头部
    {"head_yaw_joint",                  "head_yaw_joint"},
    {"head_yaw",                        "head_yaw_joint"},

    {"head_pitch_joint",                "head_pitch_joint"},
    {"head_pitch",                      "head_pitch_joint"},

    // 腰部
    {"waist_yaw_joint",                 "waist_yaw_joint"},
    {"waist_yaw",                       "waist_yaw_joint"},

    // 左手
    {"hand_l1_joint",                  "left_thumb_metacarpal_joint"},
    {"left_thumb_metacarpal_joint",     "left_thumb_metacarpal_joint"},
    {"left_thumb_metacarpal",           "left_thumb_metacarpal_joint"},

    {"hand_l2_joint",                  "left_thumb_proximal_joint"},
    {"left_thumb_proximal_joint",       "left_thumb_proximal_joint"},
    {"left_thumb_proximal",             "left_thumb_proximal_joint"},

    {"hand_l3_joint",                  "left_index_proximal_joint"},
    {"left_index_proximal_joint",       "left_index_proximal_joint"},
    {"left_index_proximal",             "left_index_proximal_joint"},
    
    {"hand_l4_joint",                  "left_index_distal_joint"},
    {"left_index_distal_joint",         "left_index_distal_joint"},
    {"left_index_distal",               "left_index_distal_joint"},

    {"hand_l5_joint",                  "left_middle_proximal_joint"},
    {"left_middle_proximal_joint",      "left_middle_proximal_joint"},
    {"left_middle_proximal",            "left_middle_proximal_joint"},

    {"hand_l6_joint",                  "left_middle_distal_joint"},
    {"left_middle_distal_joint",        "left_middle_distal_joint"},
    {"left_middle_distal",              "left_middle_distal_joint"},

    {"hand_l7_joint",                  "left_ring_proximal_joint"},
    {"left_ring_proximal_joint",        "left_ring_proximal_joint"},
    {"left_ring_proximal",              "left_ring_proximal_joint"},

    {"hand_l8_joint",                  "left_ring_distal_joint"},
    {"left_ring_distal_joint",          "left_ring_distal_joint"},
    {"left_ring_distal",                "left_ring_distal_joint"},

    {"hand_l9_joint",                  "left_pinky_proximal_joint"},
    {"left_pinky_proximal_joint",       "left_pinky_proximal_joint"},
    {"left_pinky_proximal",             "left_pinky_proximal_joint"},
    
    {"hand_l10_joint",                 "left_pinky_distal_joint"},
    {"left_pinky_distal_joint",         "left_pinky_distal_joint"},
    {"left_pinky_distal",               "left_pinky_distal_joint"},

    // 右手
    {"hand_r1_joint",                  "right_thumb_metacarpal_joint"},
    {"right_thumb_metacarpal_joint",    "right_thumb_metacarpal_joint"},
    {"right_thumb_metacarpal",          "right_thumb_metacarpal_joint"},

    {"hand_r2_joint",                  "right_thumb_proximal_joint"},
    {"right_thumb_proximal_joint",      "right_thumb_proximal_joint"},
    {"right_thumb_proximal",            "right_thumb_proximal_joint"},

    {"hand_r3_joint",                  "right_index_proximal_joint"},
    {"right_index_proximal_joint",      "right_index_proximal_joint"},
    {"right_index_proximal",            "right_index_proximal_joint"},

    {"hand_r4_joint",                  "right_index_distal_joint"},
    {"right_index_distal_joint",        "right_index_distal_joint"},
    {"right_index_distal",              "right_index_distal_joint"},

    {"hand_r5_joint",                  "right_middle_proximal_joint"},
    {"right_middle_proximal_joint",     "right_middle_proximal_joint"},
    {"right_middle_proximal",           "right_middle_proximal_joint"},

    {"hand_r6_joint",                  "right_middle_distal_joint"},
    {"right_middle_distal_joint",       "right_middle_distal_joint"},
    {"right_middle_distal",             "right_middle_distal_joint"},

    {"hand_r7_joint",                  "right_ring_proximal_joint"},
    {"right_ring_proximal_joint",       "right_ring_proximal_joint"},
    {"right_ring_proximal",             "right_ring_proximal_joint"},

    {"hand_r8_joint",                  "right_ring_distal_joint"},
    {"right_ring_distal_joint",         "right_ring_distal_joint"},
    {"right_ring_distal",               "right_ring_distal_joint"},

    {"hand_r9_joint",                  "right_pinky_proximal_joint"},
    {"right_pinky_proximal_joint",      "right_pinky_proximal_joint"},
    {"right_pinky_proximal",            "right_pinky_proximal_joint"},

    {"hand_r10_joint",                 "right_pinky_distal_joint"},
    {"right_pinky_distal_joint",        "right_pinky_distal_joint"},
    {"right_pinky_distal",              "right_pinky_distal_joint"},
};

std::string resolveJointName(std::string joint_name)
{
    auto it = joint_name_map.find(joint_name);
    if (it != joint_name_map.end())
    {
        return it->second;
    }
    return "null";
}

std::vector<std::string> getLegJointNames()
{
    std::vector<std::string> names;
    for (int i = gLegStartId; i <= gLegEndId; ++i)
    {
        names.push_back(getJointNameById(i));
    }
    return names;
}
std::vector<std::string> getArmJointNames()
{
    std::vector<std::string> names;
    for (int i = gArmStartId; i <= gArmEndId; ++i)
    {
        names.push_back(getJointNameById(i));
    }
    return names;
}
std::vector<std::string> getHeadJointNames()
{
    std::vector<std::string> names;
    for (int i = gHeadStartId; i <= gHeadEndId; ++i)
    {
        names.push_back(getJointNameById(i));
    }
    return names;
}
std::vector<std::string> getWaistJointNames()
{
    std::vector<std::string> names;
    for (int i = gWaistStartId; i <= gWaistEndId; ++i)
    {
        names.push_back(getJointNameById(i));
    }
    return names;
}
std::vector<std::string> getHandJointNames()
{
    std::vector<std::string> names;
    for (int i = gHandStartId; i <= gHandEndId; ++i)
    {   
        names.push_back(getJointNameById(i));
    }
    return names;
}
std::vector<std::string> getAllJointNames()
{
    std::vector<std::string> names;
    for (int i = 0; i < gTotalJointNum; ++i)
    {   
        names.push_back(getJointNameById(i));
    }
    return names;
}


int getJointId(std::string joint_name)
{
    auto name = resolveJointName(joint_name);
    auto it = gJointNameToId.find(name);
    if (it != gJointNameToId.end())
    {
        return it->second;
    }
    return -1;
}

std::string getJointNameById(int joint_id)
{
    if (joint_id >= 0 && joint_id < gTotalJointNum)
    {
        return kCanonicalNames[static_cast<std::size_t>(joint_id)];
    }
    return "null";
}

std::string getJointName(int joint_id)
{
    return getJointNameById(joint_id);
}

} // namespace robot
