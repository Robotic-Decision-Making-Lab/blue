#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

std::vector<double> position_to_list(const geometry_msgs::msg::Point &position) {
    return {position.x, position.y, position.z};
}

std::vector<double> quaternion_to_list(const geometry_msgs::msg::Quaternion &orientation) {
    return {orientation.x, orientation.y, orientation.z, orientation.w};
}

class MoveBluerov : public rclcpp::Node {
public:
    MoveBluerov() : Node("move_bluerov") {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/bluerov2_heavy/odometry", 10,
            std::bind(&MoveBluerov::odom_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/jt_controller/joint_trajectory", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&MoveBluerov::issue_jt_control_command, this)
        );
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_pose_position_ = position_to_list(msg->pose.pose.position);
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        odom_pose_orientation_rpy_ = {roll, pitch, yaw};
    }

    void issue_jt_control_command() {
        RCLCPP_INFO(this->get_logger(), "pose: [%f, %f, %f], [%f, %f, %f]",
                    odom_pose_position_[0], odom_pose_position_[1], odom_pose_position_[2],
                    odom_pose_orientation_rpy_[0], odom_pose_orientation_rpy_[1], odom_pose_orientation_rpy_[2]);

        std::vector<double> joint_state = odom_pose_position_;
        joint_state.insert(joint_state.end(), odom_pose_orientation_rpy_.begin(), odom_pose_orientation_rpy_.end());

        if (!joint_state.empty()) {
            trajectory_msgs::msg::JointTrajectoryPoint point1, point2;
            point1.time_from_start = rclcpp::Duration(0, 0);
            point2.time_from_start = rclcpp::Duration(4, 0);

            point1.positions = joint_state;
            std::vector<double> vel_1(6, 0.0);
            vel_1[0] = 0.2;
            point1.velocities = vel_1;

            std::vector<double> pos_2 = joint_state;
            pos_2[0] += 0.2 * 4.0;
            point2.positions = pos_2;
            std::vector<double> vel_2(6, 0.0);
            point2.velocities = vel_2;

            trajectory_msgs::msg::JointTrajectory trajectory;
            trajectory.joint_names = {"joint_x", "joint_y", "joint_z", "joint_rx", "joint_ry", "joint_rz"};
            trajectory.points = {point1, point2};
            trajectory.header.stamp = this->get_clock()->now();
            trajectory.header.frame_id = "base_link";

            publisher_->publish(trajectory);
            RCLCPP_INFO(this->get_logger(), "published trajectory");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> odom_pose_position_;
    std::vector<double> odom_pose_orientation_rpy_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveBluerov>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
