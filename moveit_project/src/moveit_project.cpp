#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_project",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_project");

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, "arm");

  sensor_msgs::msg::JointState msg;
  msg.position.push_back(3.14f);
  msg.position.push_back(1.14f);
  msg.position.push_back(1.14f);
  msg.position.push_back(2.14f);
  msg.position.push_back(2.14f);
  msg.position.push_back(1.14f);
  msg.position.push_back(0.14f);

  move_group_interface.setJointValueTarget(msg.position);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg2;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg2));
    return std::make_pair(ok, msg2);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}