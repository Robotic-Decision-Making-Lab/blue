#ifndef OMPL_PLANNER_HPP
#define OMPL_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


class MoveBluerov : public rclcpp::Node {
public:
    MoveBluerov(std::string odom_topic, std::string controller_topic, std::string node_name, std::string frame_id);
    std::string frame_id_;
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void issue_jt_control_command();
    void planner();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> odom_pose_position_;
    std::vector<double> odom_pose_orientation_rpy_;
};

#endif // OMPL_PLANNER_HPP
