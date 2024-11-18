#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "gazebo_msgs/srv/get_entity_state.hpp"
//#include <geometry_msgs/Pose.h>
#include <thread>  
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using moveit::planning_interface::MoveGroupInterface;
namespace rvt = rviz_visual_tools;


int main(int argc, char * argv[])
{

    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("show_side_grasps");

     auto const logger = rclcpp::get_logger("hello_moveit");

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Tworzymy klienta do serwisu /get_entity_state
    auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC};
    moveit_visual_tools.loadRemoteControl();
    moveit_visual_tools.deleteAllMarkers();

    // Sprawdzamy, czy serwis jest dostępny
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupt signal received, shutting down...");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /get_entity_state to be available...");
    }

    auto move_group_interface_arm = MoveGroupInterface(node, "arm");

    auto jmp = move_group_interface_arm.getRobotModel()->getJointModelGroup("gripper");



    // Tworzymy zapytanie do serwisu
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "green_cube_3";  // Nazwa obiektu w symulatorze
    request->reference_frame = "base_footprint";

  
    // Wysyłamy zapytanie i czekamy na odpowiedź
    auto result_future = client->async_send_request(request);

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


        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << -1.0, 0.0,   0.0,
                          0.0,    1.0, 0.0,
                          0.0,    0.0,   -1.0;

        Eigen::Matrix3d mat_30;
        mat_30 << -1.0, 0.0, 0.0,
                    0.0, 0.87, -0.5,
                    0.0, 0.5, -0.87;

        Eigen::Matrix3d mat_minus_30;
        mat_minus_30 << -1.0, 0.0, 0.0,
                    0.0, 0.87, -0.5,
                    0.0, 0.5, -0.87;

        // Konwersja macierzy rotacji na kwaternion

        std::vector<moveit_msgs::msg::Grasp> vis_grasps;


        for (int j = 0; j < 4; j++)
        {

            for (int m = 0; m < 3; m++){

                for (int i = 0; i < 3; i++){

                    Eigen::Quaterniond quaternion(1,0,0,0);
                    quaternion.normalize();
                    
                    if (m == 0){
                        quaternion = quaternion * Eigen::Quaterniond(rotation_matrix);
                    }
                    else if (m == 1){
                        quaternion = quaternion * Eigen::Quaterniond(mat_30);
                    }
                    else{
                        quaternion = quaternion * Eigen::Quaterniond(mat_minus_30);
                    }

                    quaternion.normalize();

                    moveit_msgs::msg::Grasp vis_gr;
                    
                    vis_gr.grasp_pose.header.frame_id = "base_footprint";

                    geometry_msgs::msg::Pose pose_msg ;
                    pose_msg.position.x = pose.position.x;
                    pose_msg.position.y = pose.position.y;

                    if (i == 0){
                        if (m == 0){
                            pose_msg.position.z = pose.position.z+0.3;
                        }
                        else{
                            pose_msg.position.z = pose.position.z+0.15;//0.3*cos30

                            if (m == 1)
                            {
                                pose_msg.position.y = pose.position.y + 0.26;
                            }
                            else if (m==2)
                            {
                                pose_msg.position.y = pose.position.y - 0.26;
                            }
                        }
                        vis_gr.id = "grasp_1";

                        pose_msg.orientation.x = quaternion.x();
                        pose_msg.orientation.y = quaternion.y();
                        pose_msg.orientation.z = quaternion.z();
                        pose_msg.orientation.w = quaternion.w();
                    }
                    else if (i == 1){

                        if (m == 0){
                            pose_msg.position.z = pose.position.z+0.22;
                        }
                        else{
                            pose_msg.position.z = pose.position.z+0.11;//0.3*cos30

                            if (m == 1)
                            {
                                pose_msg.position.y = pose.position.y + 0.19;
                            }
                            else if (m==2)
                            {
                                pose_msg.position.y = pose.position.y - 0.19;
                            }
                        }
                        vis_gr.id = "grasp_2";

                        pose_msg.orientation.x = quaternion.x();
                        pose_msg.orientation.y = quaternion.y();
                        pose_msg.orientation.z = quaternion.z();
                        pose_msg.orientation.w = quaternion.w();

                    }
                    else{

                        pose_msg.position.z = pose.position.z;
                        vis_gr.id = "grasp_3";

                        pose_msg.orientation.x = pose.orientation.x;
                        pose_msg.orientation.y = pose.orientation.y;
                        pose_msg.orientation.z = pose.orientation.z;
                        pose_msg.orientation.w = pose.orientation.w;
                    }

                    vis_gr.grasp_pose.pose = pose_msg;
                    moveit_visual_tools.publishAxis(pose_msg);
                    vis_grasps.push_back(vis_gr);
                    
                }
                moveit_visual_tools.publishGrasps(vis_grasps, jmp);
                moveit_visual_tools.trigger();
                moveit_visual_tools.prompt("Next");
                moveit_visual_tools.trigger();
                moveit_visual_tools.deleteAllMarkers();
            }
        }

    }                

    rclcpp::shutdown();
    return 0;
}