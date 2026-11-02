#include <chrono>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <crb_ros_msg/msg/joint_state_data.hpp>

#include <rl_controller/rl_interface.hpp>

std::shared_ptr<rclcpp::Node> global_node_ptr;

std::shared_ptr<sensor_msgs::msg::JointState> joint_data_in_ptr = nullptr;
std::shared_ptr<sensor_msgs::msg::JointState> debug_joint_data_in_ptr = nullptr;
std::chrono::steady_clock::time_point joint_data_recv{};
std::chrono::steady_clock::time_point debug_joint_data_recv{};

std::shared_ptr<sensor_msgs::msg::Imu> imu_data_ptr = nullptr;

std::shared_ptr<geometry_msgs::msg::Twist> cmd_twist_ptr = nullptr;

int gRobotMode = 0;

std::mutex joint_data_in_mutex;
std::mutex imu_data_mutex;
std::mutex robot_mode_mutex;
std::mutex cmd_twist_mutex;

sensor_msgs::msg::JointState toJointState(const crb_ros_msg::msg::JointStateData &msg)
{
    sensor_msgs::msg::JointState out;
    out.header = msg.header;
    out.name = msg.name;
    out.position = msg.position;
    out.velocity = msg.velocity;
    out.effort = msg.effort;
    return out;
}

crb_ros_msg::msg::JointStateData toDebugJointCmd(const sensor_msgs::msg::JointState &msg,
                                                 const message::outputStruct &out_data)
{
    crb_ros_msg::msg::JointStateData out;
    out.header = msg.header;
    out.name = msg.name;
    out.position = msg.position;
    out.velocity = msg.velocity;
    out.effort.assign(msg.name.size(), 0.0);
    robot::getPdGainsFromOutputStruct(out_data, out.kp, out.kd);
    return out;
}

void handleJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_data_in_mutex);
    joint_data_in_ptr = msg;
    joint_data_recv = std::chrono::steady_clock::now();
}

void handleDebugJointState(const crb_ros_msg::msg::JointStateData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_data_in_mutex);
    debug_joint_data_in_ptr = std::make_shared<sensor_msgs::msg::JointState>(toJointState(*msg));
    debug_joint_data_recv = std::chrono::steady_clock::now();
}

void handleImuState(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data_ptr = msg;
}

void handleRobotMode(const std_msgs::msg::Int32::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(robot_mode_mutex);
    gRobotMode = msg->data;
}

void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cmd_twist_mutex);
    cmd_twist_ptr = msg;
}

int main(int argc, char *argv[])
{
    // Value-initialize every scalar/array field. Until the first valid IMU
    // arrives, use the identity quaternion instead of an indeterminate value.
    message::inputStruct in_data{};
    message::outputStruct out_data{};
    in_data.orientation[0] = 1.0;

    rclcpp::init(argc, argv);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_publisher_;
    rclcpp::Publisher<crb_ros_msg::msg::JointStateData>::SharedPtr debug_joint_cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<crb_ros_msg::msg::JointStateData>::SharedPtr debug_joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_mode_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    global_node_ptr = std::make_shared<rclcpp::Node>("rl_controller_test_node");

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                                 .reliability(rclcpp::ReliabilityPolicy::Reliable)
                                 .durability(rclcpp::DurabilityPolicy::Volatile);
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                               .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                               .durability(rclcpp::DurabilityPolicy::Volatile);

    joint_cmd_publisher_ = global_node_ptr->create_publisher<sensor_msgs::msg::JointState>(
        "/motion/joint_cmd", command_qos);
    debug_joint_cmd_publisher_ = global_node_ptr->create_publisher<crb_ros_msg::msg::JointStateData>(
        "/motion/debug/joint_cmd", command_qos);

    debug_joint_state_subscriber_ =
        global_node_ptr->create_subscription<crb_ros_msg::msg::JointStateData>(
            "/motion/debug/joint_state", state_qos,
            std::bind(&handleDebugJointState, std::placeholders::_1));

    joint_state_subscriber_ = global_node_ptr->create_subscription<sensor_msgs::msg::JointState>(
        "/motion/joint_state", state_qos,
        std::bind(&handleJointState, std::placeholders::_1));

    imu_state_subscriber_ = global_node_ptr->create_subscription<sensor_msgs::msg::Imu>(
        "/motion/imu", state_qos,
        std::bind(&handleImuState, std::placeholders::_1));

    robot_mode_subscriber_ = global_node_ptr->create_subscription<std_msgs::msg::Int32>(
        "/motion/current_mode", command_qos,
        std::bind(&handleRobotMode, std::placeholders::_1));

    // subscribe cmd vel
    cmd_vel_subscriber_ = global_node_ptr->create_subscription<geometry_msgs::msg::Twist>(
        "/motion/cmd_vel", command_qos,
        std::bind(&handleCmdVel, std::placeholders::_1));

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(global_node_ptr);
    std::thread spin_thread([&executor]() { executor.spin(); });

    std::shared_ptr<rl::RlInterface> rlInterface = std::make_shared<rl::RlInterface>();

    // 循环
    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(cmd_twist_mutex);
            if(cmd_twist_ptr)
            {
                in_data.cmdVx = cmd_twist_ptr->linear.x;
                in_data.cmdVy = cmd_twist_ptr->linear.y;
                in_data.cmdWz = cmd_twist_ptr->angular.z;
            }
            else
            {
                in_data.cmdVx = 0.0;
                in_data.cmdVy = 0.0;
                in_data.cmdWz = 0.0;
            }
        }
        
        rlInterface->robot_mode = gRobotMode;

        bool jointDataReady = false;
        {
            std::lock_guard<std::mutex> lock(joint_data_in_mutex);
            const sensor_msgs::msg::JointState *latest = nullptr;
            if (debug_joint_data_in_ptr && joint_data_in_ptr)
            {
                latest = (debug_joint_data_recv >= joint_data_recv)
                             ? debug_joint_data_in_ptr.get()
                             : joint_data_in_ptr.get();
            }
            else if (debug_joint_data_in_ptr)
            {
                latest = debug_joint_data_in_ptr.get();
            }
            else if (joint_data_in_ptr)
            {
                latest = joint_data_in_ptr.get();
            }
            if (latest)
            {
                jointDataReady = robot::setJointDataInInputStruct(*latest, in_data);
            }
        }

        bool imuDataReady = false;
        {
            std::lock_guard<std::mutex> lock(imu_data_mutex);
            if (imu_data_ptr)
            {
                imuDataReady =
                    robot::setImuDataInInputStruct(*imu_data_ptr, in_data);
            }
        }

        // Do not start policy inference or publish commands until both sensor
        // sources have provided a complete first frame.
        if (!jointDataReady || !imuDataReady)
        {
            rate.sleep();
            continue;
        }

        // RCLCPP_INFO(rclcpp::get_logger("rl_handler_node"), "in_data j[0]=%.3f, arm[0]=%.3f", in_data.leg_data[0].pos_real, in_data.arm_data[0].pos_real);
        rlInterface->update(in_data, out_data);
        // RCLCPP_INFO(rclcpp::get_logger("rl_handler_node"), "output j[0]=%.3f, arm[0]=%.3f", out_data.leg_cmd[0].pos_des, out_data.arm_cmd[0].pos_des);

        sensor_msgs::msg::JointState joint_data_out;
        robot::getJointDataFromOutputStruct(out_data, joint_data_out);
        joint_data_out.header.stamp = global_node_ptr->now();
        joint_cmd_publisher_->publish(joint_data_out);
        debug_joint_cmd_publisher_->publish(toDebugJointCmd(joint_data_out, out_data));

        rate.sleep();
    }

    executor.cancel();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }
    executor.remove_node(global_node_ptr);
    rclcpp::shutdown();
    return 0;
}
