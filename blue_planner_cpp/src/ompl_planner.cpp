#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <math.h>

std::vector<double> position_to_list(const geometry_msgs::msg::Point &position) {
    return {position.x, position.y, position.z};
}

std::vector<double> quaternion_to_list(const geometry_msgs::msg::Quaternion &orientation) {
    return {orientation.x, orientation.y, orientation.z, orientation.w};
}

class MoveBluerov : public rclcpp::Node {
public:
    MoveBluerov(std::string odom_topic, std::string controller_topic, std::string node_name, std::string frame_id) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "MoveBluerov node started");
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&MoveBluerov::odom_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            controller_topic, 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&MoveBluerov::issue_jt_control_command, this)
        );
        frame_id_ = frame_id;
    }

    void planner(std::vector<double> start_state, std::vector<double> goal_state) {
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(6));

        ompl::base::RealVectorBounds bounds(6);
        bounds.setLow(0, -100);
        bounds.setHigh(0, 100);
        bounds.setLow(1, -100);
        bounds.setHigh(1, 100);
        bounds.setLow(2, -100);
        bounds.setHigh(2, 100);
        bounds.setLow(3, -M_PI);
        bounds.setHigh(3, M_PI);
        bounds.setLow(4, -M_PI);
        bounds.setHigh(4, M_PI);
        bounds.setLow(5, -M_PI);
        bounds.setHigh(5, M_PI);

        space->setBounds(bounds);

        ompl::geometric::SimpleSetup ss(space);

        ss.setStateValidityChecker([](const ompl::base::State *state) {
            const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
            // return (pos->values[0] * pos->values[0] + pos->values[1] * pos->values[1] < 1);
            return (pos->values[0] >= -100 && pos->values[0] <= 100 &&
                    pos->values[1] >= -100 && pos->values[1] <= 100 &&
                    pos->values[2] >= -100 && pos->values[2] <= 100 &&
                    pos->values[3] >= -M_PI && pos->values[3] <= M_PI &&
                    pos->values[4] >= -M_PI && pos->values[4] <= M_PI &&
                    pos->values[5] >= -M_PI && pos->values[5] <= M_PI);
        });

        ompl::base::ScopedState<> start(space);
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_state[0];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_state[1];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_state[2];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = start_state[3];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = start_state[4];
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = start_state[5];

        ompl::base::ScopedState<> goal(space);
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = goal_state[0];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = goal_state[1];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = goal_state[2];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = goal_state[3];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = goal_state[4];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = goal_state[5];

        ss.setStartAndGoalStates(start, goal);

        ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
        ompl::base::PlannerStatus solved = ss.solve(1.0);

    if (solved) {
        RCLCPP_INFO(this->get_logger(), "Found solution:");
        ompl::geometric::PathGeometric path = ss.getSolutionPath();
        get_trajectory(path);
    } else {
        RCLCPP_INFO(this->get_logger(), "No solution found");
    }
    }

    void print_trajectory(ompl::geometric::PathGeometric path) {
        for (std::size_t i = 0; i < path.getStateCount(); ++i) {
            const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            RCLCPP_INFO(this->get_logger(), "State %zu: [%f, %f, %f, %f, %f, %f]", i,
                        state->values[0], state->values[1], state->values[2],
                        state->values[3], state->values[4], state->values[5]);
        }
    }

    void get_trajectory(ompl::geometric::PathGeometric path) {
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {"joint_x", "joint_y", "joint_z", "joint_rx", "joint_ry", "joint_rz"};
        for (std::size_t i = 0; i < path.getStateCount(); ++i) {
            const auto *state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration(4*i, 0);
            point.positions = {state->values[0], state->values[1], state->values[2],
                               state->values[3], state->values[4], state->values[5]};
            trajectory.points.push_back(point);
        }
        trajectory.header.stamp = this->get_clock()->now();
        trajectory.header.frame_id = frame_id_;
        publisher_->publish(trajectory);
        RCLCPP_INFO(this->get_logger(), "published trajectory");
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

        std::vector<double> start_state, goal_state;
        if (!joint_state.empty()) {
            start_state = joint_state;
            goal_state = joint_state;
            goal_state[0] += 0.2 * 4.0;
            goal_state[1] += 0.2 * 4.0;
            goal_state[2] += 0.2 * 4.0;
            RCLCPP_INFO(this->get_logger(), "start: [%f, %f, %f], goal: [%f, %f, %f]",
                        start_state[0], start_state[1], start_state[2],
                        goal_state[0], goal_state[1], goal_state[2]);

            planner(start_state, goal_state);
            // trajectory_msgs::msg::JointTrajectoryPoint point1, point2;
            // point1.time_from_start = rclcpp::Duration(0, 0);
            // point2.time_from_start = rclcpp::Duration(4, 0);

            // point1.positions = joint_state;
            // std::vector<double> vel_1(6, 0.0);
            // vel_1[0] = 0.2;
            // point1.velocities = vel_1;

            // std::vector<double> pos_2 = joint_state;
            // pos_2[0] += 0.2 * 4.0;
            // point2.positions = pos_2;
            // std::vector<double> vel_2(6, 0.0);
            // point2.velocities = vel_2;

            // trajectory_msgs::msg::JointTrajectory trajectory;
            // trajectory.joint_names = {"joint_x", "joint_y", "joint_z", "joint_rx", "joint_ry", "joint_rz"};
            // trajectory.points = {point1, point2};
            // trajectory.header.stamp = this->get_clock()->now();
            // trajectory.header.frame_id = "base_link";

            // publisher_->publish(trajectory);
            // RCLCPP_INFO(this->get_logger(), "published trajectory");
        }
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> odom_pose_position_;
    std::vector<double> odom_pose_orientation_rpy_;
    std::string frame_id_;

};

// int main(int argc, char **argv) {
//     #include <rclcpp/rclcpp.hpp>
// #include <memory>

// Assuming MoveBluerov is your custom node class
// #include "your_package/move_bluerov.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create the first node
    std::string odom_topic = "/model/rob_1/bluerov2_heavy/odometry";
    std::string controller_topic = "/rob_1/jt_controller/joint_trajectory";
    auto node = std::make_shared<MoveBluerov>(odom_topic, controller_topic, "r1_move_node", "rob_1/base_link");

    // Create the second node
    std::string odom_topic_2 = "/model/rob_3/bluerov2_heavy/odometry";
    std::string controller_topic_2 = "/rob_3/jt_controller/joint_trajectory";
    auto node_2 = std::make_shared<MoveBluerov>(odom_topic_2, controller_topic_2, "r3_move_node", "rob_3/base_link");

    // Create an executor and add both nodes to it
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node_2);

    // Spin the executor - this will handle callbacks for both nodes
    executor.spin();

    // If you need to call specific methods on your nodes:
    // This can be done before spinning the executor if it's a one-time setup
    // node->planner();

    // Or you can use a separate thread if you need to call methods while the executor is spinning
    // std::thread planner_thread([&node]() {
    //   node->planner();
    // });

    // Cleanup
    rclcpp::shutdown();

    // If you used the thread approach:
    // planner_thread.join();

    return 0;
  }


// #include "blue_planner_cpp/ompl_planner.hpp"
// #include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>

// std::vector<double> position_to_list(const geometry_msgs::msg::Point &position) {
//     return {position.x, position.y, position.z};
// }

// std::vector<double> quaternion_to_list(const geometry_msgs::msg::Quaternion &orientation) {
//     return {orientation.x, orientation.y, orientation.z, orientation.w};
// }

// MoveBluerov::MoveBluerov() : Node("move_bluerov") {
//     subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/model/bluerov2_heavy/odometry", 10,
//         std::bind(&MoveBluerov::odom_callback, this, std::placeholders::_1)
//     );

//     publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//         "/jt_controller/joint_trajectory", 10
//     );

//     timer_ = this->create_wall_timer(
//         std::chrono::seconds(2),
//         std::bind(&MoveBluerov::issue_jt_control_command, this)
//     );
// }

// void MoveBluerov::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     odom_pose_position_ = position_to_list(msg->pose.pose.position);
//     tf2::Quaternion q(
//         msg->pose.pose.orientation.x,
//         msg->pose.pose.orientation.y,
//         msg->pose.pose.orientation.z,
//         msg->pose.pose.orientation.w
//     );
//     tf2::Matrix3x3 m(q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     odom_pose_orientation_rpy_ = {roll, pitch, yaw};
// }

// void MoveBluerov::issue_jt_control_command() {
//     RCLCPP_INFO(this->get_logger(), "pose: [%f, %f, %f], [%f, %f, %f]",
//                 odom_pose_position_[0], odom_pose_position_[1], odom_pose_position_[2],
//                 odom_pose_orientation_rpy_[0], odom_pose_orientation_rpy_[1], odom_pose_orientation_rpy_[2]);

//     std::vector<double> joint_state = odom_pose_position_;
//     joint_state.insert(joint_state.end(), odom_pose_orientation_rpy_.begin(), odom_pose_orientation_rpy_.end());

//     if (!joint_state.empty()) {
//         trajectory_msgs::msg::JointTrajectoryPoint point1, point2;
//         point1.time_from_start = rclcpp::Duration(0, 0);
//         point2.time_from_start = rclcpp::Duration(4, 0);

//         point1.positions = joint_state;
//         std::vector<double> vel_1(6, 0.0);
//         vel_1[0] = 0.2;
//         point1.velocities = vel_1;

//         std::vector<double> pos_2 = joint_state;
//         pos_2[0] += 0.2 * 4.0;
//         point2.positions = pos_2;
//         std::vector<double> vel_2(6, 0.0);
//         point2.velocities = vel_2;

//         trajectory_msgs::msg::JointTrajectory trajectory;
//         trajectory.joint_names = {"joint_x", "joint_y", "joint_z", "joint_rx", "joint_ry", "joint_rz"};
//         trajectory.points = {point1, point2};
//         trajectory.header.stamp = this->get_clock()->now();
//         trajectory.header.frame_id = "base_link";

//         publisher_->publish(trajectory);
//         RCLCPP_INFO(this->get_logger(), "published trajectory");
//     }
// }

// void MoveBluerov::planner() {
//     auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

//     ompl::base::RealVectorBounds bounds(2);
//     bounds.setLow(-1);
//     bounds.setHigh(1);

//     space->setBounds(bounds);

//     ompl::geometric::SimpleSetup ss(space);

//     ss.setStateValidityChecker([](const ompl::base::State *state) {
//         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
//         return (pos->values[0] * pos->values[0] + pos->values[1] * pos->values[1] < 1);
//     });

//     ompl::base::ScopedState<> start(space);
//     start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -0.5;
//     start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = -0.5;

//     ompl::base::ScopedState<> goal(space);
//     goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.5;
//     goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0.5;

//     ss.setStartAndGoalStates(start, goal);

//     ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));

//     ompl::base::PlannerStatus solved = ss.solve(1.0);

//     if (solved)
//     {
//         ss.simplifySolution();
//         ss.getSolutionPath().print(std::cout);
//     }
//     else
//     {
//         std::cout << "No solution found" << std::endl;
//     }
// }

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveBluerov>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
