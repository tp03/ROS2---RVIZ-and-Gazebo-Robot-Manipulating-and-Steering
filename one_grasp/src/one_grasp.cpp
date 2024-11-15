#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include <thread>  
#include <string>

using moveit::planning_interface::MoveGroupInterface;
namespace rvt = rviz_visual_tools;

int main(int argc, char * argv[])
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("one_grasp");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // Create the MoveIt MoveGroup Interface
    auto move_group_interface = MoveGroupInterface(node, "arm_torso");
    // move_group_interface.setMaxVelocityScalingFactor(0.5);  // Zwiększamy prędkość
    // move_group_interface.setMaxAccelerationScalingFactor(0.5);  // Zwiększamy przyspieszenie
    // auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");
    // std::string end_effector_link = "wrist_ft_link";  // Zmienna z nazwą linku nadgarstka
    // move_group_interface.setEndEffectorLink(end_effector_link);

    // Tworzymy klienta do serwisu /get_entity_state
    auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

    // Sprawdzamy, czy serwis jest dostępny
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupt signal received, shutting down...");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /get_entity_state to be available...");
    }

    // Tworzymy obiekt MoveItVisualTools do wyświetlania w RViz
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Tworzymy zapytanie do serwisu
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "green_cube_3";  // Nazwa obiektu w symulatorze
    request->reference_frame = "base_footprint";

  
    // Wysyłamy zapytanie i czekamy na odpowiedź
    auto result_future = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto &pose = result_future.get()->state.pose;
        RCLCPP_INFO(node->get_logger(), "Successfully retrieved state of entity 'green_cube_3'");
        RCLCPP_INFO(node->get_logger(), "Position: x = %.2f, y = %.2f, z = %.2f", 
                   pose.position.x, 
                   pose.position.y, 
                   pose.position.z);
        RCLCPP_INFO(node->get_logger(), "Orientation: x = %.2f, y = %.2f, z = %.2f, w = %.2f",
                    pose.orientation.x, 
                    pose.orientation.y, 
                    pose.orientation.z, 
                    pose.orientation.w);

        moveit_visual_tools.publishCuboid(pose, 0.7, 0.7, 0.7, rvt::GREEN);

        // Wyświetlenie markerów
        moveit_visual_tools.trigger();}
    else {
      RCLCPP_ERROR(node->get_logger(), "Service call failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
