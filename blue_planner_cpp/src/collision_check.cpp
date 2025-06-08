#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <future>

void perform_collision_checking(const pinocchio::Model &model, const pinocchio::GeometryModel &collision_model)
{
  pinocchio::Data data(model);
  pinocchio::GeometryData collision_data(collision_model);

  // Get a joint configuration.
  Eigen::VectorXd q(model.nq);
  q << 0.0, -0.785, -3.14, -1.57, 1.57, 0.0;
  std::cout << "Joint configuration: " << std::endl << q << std::endl << std::endl;

  // Get the frame ID of the end effector for later lookups.
  const auto ee_frame_id = model.getFrameId("rob_1/base_link");

  // Perform forward kinematics and get a transform.
  pinocchio::framesForwardKinematics(model, data, q);
  std::cout << "Frame transform: " << std::endl << data.oMf[ee_frame_id] << std::endl;

  // Get a Jacobian at a specific frame.
  Eigen::MatrixXd ee_jacobian(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, ee_jacobian);
  std::cout << "Frame Jacobian: " << std::endl << ee_jacobian << std::endl << std::endl;

  // Check collisions.
  pinocchio::computeCollisions(model, data, collision_model, collision_data, q);
  for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
  {
    const auto &collision_pair = collision_model.collisionPairs[k];
    if (collision_data.collisionResults[k].isCollision())
    {
      std::cout << "Collision detected between " << collision_model.geometryObjects[collision_pair.first].name
                << " and " << collision_model.geometryObjects[collision_pair.second].name << std::endl;
    }
  }
}

void process_urdf(const std::string &urdf_xml)
{
  // Save the URDF XML to a file
  std::ofstream urdf_file("/tmp/robot_description.urdf");
  if (urdf_file.is_open())
  {
    urdf_file << urdf_xml;
    urdf_file.close();
    RCLCPP_INFO(rclcpp::get_logger("urdf_subscriber"), "URDF XML saved to /tmp/robot_description.urdf");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("urdf_subscriber"), "Failed to open file for writing URDF XML");
    return;
  }

  // Create a set of Pinocchio models and data.
  pinocchio::Model model;
  pinocchio::urdf::buildModel("/tmp/robot_description.urdf", model);

  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model, "/tmp/robot_description.urdf", pinocchio::VISUAL, visual_model);

  pinocchio::GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model, "/tmp/robot_description.urdf", pinocchio::COLLISION, collision_model);

  perform_collision_checking(model, collision_model);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("urdf_subscriber");

  const auto timeout = std::chrono::seconds(1);
  std::promise<std::string> xml_promise;
  std::shared_future<std::string> xml_future(xml_promise.get_future());

  std::function<void(const std_msgs::msg::String::SharedPtr)> fun =
    [&xml_promise](const std_msgs::msg::String::SharedPtr msg) {
      xml_promise.set_value(msg->data);
    };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subs;
  description_subs = node->create_subscription<std_msgs::msg::String>(
    "/rob_1/robot_description", rclcpp::QoS(1).transient_local(), fun);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor until the URDF XML is received or timeout occurs
  if (executor.spin_until_future_complete(xml_future, timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::string urdf_xml = xml_future.get();
    RCLCPP_INFO(node->get_logger(), "Received URDF XML");
    process_urdf(urdf_xml);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to receive URDF XML within the timeout period");
  }

  rclcpp::shutdown();
  return 0;
}

// #include <filesystem>

// #include "ament_index_cpp/get_package_share_directory.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/geometry.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/parsers/srdf.hpp"
// #include "rclcpp/rclcpp.hpp"

// int main(int /*argc*/, char* /*argv*/ [])
// {
//   // Get the URDF and SRDF file paths.
//   const auto package_share_path = ament_index_cpp::get_package_share_directory("learn_ros2");
//   const auto urdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.urdf";
//   const auto srdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.srdf";

//   // Create a set of Pinocchio models and data.
//   pinocchio::Model model;
//   pinocchio::urdf::buildModel(urdf_path, model);

//   pinocchio::GeometryModel visual_model;
//   pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, visual_model);

//   pinocchio::GeometryModel collision_model;
//   pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, collision_model);
//   collision_model.addAllCollisionPairs();
//   pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_path);

//   pinocchio::Data data(model);
//   pinocchio::GeometryData collision_data(collision_model);

//   // Get a joint configuration.
//   Eigen::VectorXd q(model.nq);
//   q << 0.0, -0.785, -3.14, -1.57, 1.57, 0.0;
//   std::cout << "Joint configuration: " << std::endl << q << std::endl << std::endl;

//   // Get the frame ID of the end effector for later lookups.
//   const auto ee_frame_id = model.getFrameId("ee_link");

//   // Perform forward kinematics and get a transform.
//   pinocchio::framesForwardKinematics(model, data, q);
//   std::cout << "Frame transform: " << std::endl << data.oMf[ee_frame_id] << std::endl;

//   // Get a Jacobian at a specific frame.
//   Eigen::MatrixXd ee_jacobian(6, model.nv);
//   pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, ee_jacobian);
//   std::cout << "Frame Jacobian: " << std::endl << ee_jacobian << std::endl << std::endl;

//   // Check collisions.
//   pinocchio::computeCollisions(model, data, collision_model, collision_data, q);
//   for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
//   {
//     const auto& cp = collision_model.collisionPairs[k];
//     const auto& cr = collision_data.collisionResults[k];
//     if (cr.isCollision())
//     {
//       const auto& body1 = collision_model.geometryObjects[cp.first].name;
//       const auto& body2 = collision_model.geometryObjects[cp.second].name;
//       std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
//     }
//   }

//   return 0;
// }
