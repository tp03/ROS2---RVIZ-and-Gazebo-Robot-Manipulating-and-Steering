#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include <thread>  
#include <string>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "one_grasp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("one_grasp");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  auto move_group_interface = MoveGroupInterface(node, "arm");

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  
  std::cout<<move_group_interface.getEndEffectorLink()<<std::endl;

  // Create the GetEntityState client
  auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting...");
      return 0;
    }
    RCLCPP_INFO(logger, "Waiting for /get_entity_state service to be available...");
  }

  // Prepare the request
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = "green_cube_3";           // Name of the cube in the Gazebo simulation
  request->reference_frame = "base_link";       // Reference frame, "world" is usually the default

  // Call the service and get the result
  auto result_future = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    if (response->success) {
      RCLCPP_INFO(logger, "Successfully retrieved state of entity 'green_cube_3'");
      RCLCPP_INFO(logger, "Position: x = %.2f, y = %.2f, z = %.2f", 
                  response->state.pose.position.x, 
                  response->state.pose.position.y, 
                  response->state.pose.position.z);
      RCLCPP_INFO(logger, "Orientation: x = %.2f, y = %.2f, z = %.2f, w = %.2f",
                  response->state.pose.orientation.x, 
                  response->state.pose.orientation.y, 
                  response->state.pose.orientation.z, 
                  response->state.pose.orientation.w);
    } else {
      RCLCPP_ERROR(logger, "Failed to retrieve state for 'green_cube_3'");
    }
  } else {
    RCLCPP_ERROR(logger, "Service call failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
