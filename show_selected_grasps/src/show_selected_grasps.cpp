#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include <thread>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose.hpp>


using namespace Eigen;
using moveit::planning_interface::MoveGroupInterface;
namespace rvt = rviz_visual_tools;

int main(int argc, char * argv[])
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);

    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("show_selected_grasps");

    // Tworzymy klienta do serwisu /get_entity_state
    auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

    // Tworzymy MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC};
    moveit_visual_tools.loadRemoteControl();
    moveit_visual_tools.deleteAllMarkers();

    // Czekamy na dostępność usługi
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupt signal received, shutting down...");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /get_entity_state to be available...");
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto move_group_interface_arm = MoveGroupInterface(node, "arm");
    auto jmp = move_group_interface_arm.getRobotModel()->getJointModelGroup("gripper");

    // Tworzymy executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Wysyłamy zapytanie do serwisu
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = "green_cube_3";  // Nazwa obiektu w symulatorze
    request->reference_frame = "base_footprint";

    auto result_future = client->async_send_request(request);

    if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready){
        auto &pose = result_future.get()->state.pose;
        RCLCPP_INFO(node->get_logger(), "Successfully retrieved state of entity 'green_cube_3'");
        RCLCPP_INFO(node->get_logger(), "Position: x = %.2f, y = %.2f, z = %.2f",
                    pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(node->get_logger(), "Orientation: x = %.2f, y = %.2f, z = %.2f, w = %.2f",
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        Eigen::Isometry3d nothing(AngleAxisd(0.0, Vector3d::UnitX()));
        Eigen::Isometry3d rot_z_90(AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitZ()));
        Eigen::Isometry3d rot_y_180(AngleAxisd(M_PI, Vector3d::UnitY()));
        Eigen::Isometry3d rot_x_30(AngleAxisd(30.0/180.0*M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_minus_30(AngleAxisd(-30.0/180.0*M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_90(AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_minus_90(AngleAxisd(-90.0/180.0*M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_y_90(AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitY()));
        Eigen::Isometry3d rot_y_minus_90(AngleAxisd(-90.0/180.0*M_PI, Vector3d::UnitY()));                                   

        std::vector<moveit_msgs::msg::Grasp> vis_grasps;

        geometry_msgs::msg::Pose saved_pose = pose;


        for (int w = 0; w < 5; w++){

            Eigen::Isometry3d which_side;

            if (w == 0)
            {
                which_side = nothing;
            }
            else if (w == 1)
            {
                which_side = rot_x_90;
            }
            else if (w == 2)
            {
                which_side = rot_x_minus_90;
            }
            else if (w == 3)
            {
                which_side = rot_y_90;
            }
            else if (w == 4)
            {
                which_side = rot_y_minus_90;
            }
            for (int j = 0; j < 4; j++) {
                Eigen::Isometry3d bambi;
                if (j == 0)
                {
                    bambi = nothing;
                }
                else if (j == 1)
                {
                    bambi = rot_z_90;
                }
                else if (j == 2)
                {
                    bambi = rot_z_90*rot_z_90;
                }
                else
                {
                    bambi = rot_z_90*rot_z_90*rot_z_90;
                }

                //rot_z_90 = rot_z_90*rot_y_minus_90; 

                for (int m = 0; m < 3; m++) {

                    float pre_grasp = 0.0;
                    float grasp = 0.0;
                    geometry_msgs::msg::Pose grasp_msg;
                    for (int i = 0; i < 3; i++) {

                        
                        geometry_msgs::msg::Pose pose_msg;
                        
                        if (i == 0) {
                            if (m == 0) {
                                float _x = saved_pose.position.x;
                                float _z = saved_pose.position.z + 0.3;
                                float _y = saved_pose.position.y;
                                if (w == 0){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 1){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.3, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 2){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.3, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 3){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.3, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 4){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.3, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                            }
                            else {
                                float _z  = saved_pose.position.z + 0.26;
                                if (m == 1) {
                                    if (j == 0 || j == 2){
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y + 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.26, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.26, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y+0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y+0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _y = saved_pose.position.y - 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.26, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.26, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y-0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y-0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        float _y = saved_pose.position.y;
                                        if (j == 1)
                                        {
                                            float _x = saved_pose.position.x + 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.15, saved_pose.position.y-0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.15, saved_pose.position.y+0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }

                                        }
                                        else {
                                            float _x = saved_pose.position.x - 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.15, saved_pose.position.y-0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.15, saved_pose.position.y+0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                                else if (m == 2) {
                                    if (j == 0 || j == 2){
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y - 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.26, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.26, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y-0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y-0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _y = saved_pose.position.y + 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.26, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.26, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y+0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y+0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        float _y = saved_pose.position.y;
                                        if (j == 1)
                                        {
                                            float _x = saved_pose.position.x - 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.15, saved_pose.position.y-0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.15, saved_pose.position.y+0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _x = saved_pose.position.x + 0.15;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.15, saved_pose.position.y-0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.15, saved_pose.position.y+0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.26, saved_pose.position.y, saved_pose.position.z-0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.26, saved_pose.position.y, saved_pose.position.z+0.15));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                            }
                            
                        }
                        else if (i == 1) {
                            if (m == 0) {
                                float _x = saved_pose.position.x;
                                float _z = saved_pose.position.z + 0.22;
                                float _y = saved_pose.position.y;

                                 if (w == 0){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 1){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.22, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 2){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.22, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 3){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.22, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 4){
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.22, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_y_minus_90;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                            }
                            else {
                                float _z = saved_pose.position.z + 0.19;
                                if (m == 1) {
                                    if (j == 0 || j == 2){
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y + 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.19, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.19, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y+0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y+0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _y = saved_pose.position.y - 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.19, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.19, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y-0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y-0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        float _y = saved_pose.position.y;
                                        if (j == 1)
                                        {
                                            float _x = saved_pose.position.x + 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.11, saved_pose.position.y-0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.11, saved_pose.position.y+0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _x = saved_pose.position.x - 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.11, saved_pose.position.y-0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.11, saved_pose.position.y+0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                                else if (m == 2) {
                                    if (j == 0 || j == 2){
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y - 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.19, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.19, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y-0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y-0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _y = saved_pose.position.y + 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y-0.19, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y+0.19, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y+0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y+0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                    else
                                    {
                                        float _y = saved_pose.position.y;
                                        if (j == 1)
                                        {
                                            float _x = saved_pose.position.x - 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.11, saved_pose.position.y-0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.11, saved_pose.position.y+0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else {
                                            float _x = saved_pose.position.x + 0.11;
                                            if (w == 0){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.11, saved_pose.position.y-0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.11, saved_pose.position.y+0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x+0.19, saved_pose.position.y, saved_pose.position.z-0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4){
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x-0.19, saved_pose.position.y, saved_pose.position.z+0.11));
                                                Eigen::Isometry3d skolim = tr*which_side*rot_y_180*bambi*rot_x_minus_30*rot_z_90*rot_y_minus_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                            }
    
                        }
                        else {
                            pose_msg.position.x = saved_pose.position.x;
                            pose_msg.position.y = saved_pose.position.y;
                            pose_msg.position.z = saved_pose.position.z;
                            

                            pose_msg.orientation.x = saved_pose.orientation.x;
                            pose_msg.orientation.y = saved_pose.orientation.y;
                            pose_msg.orientation.z = saved_pose.orientation.z;
                            pose_msg.orientation.w = saved_pose.orientation.w;
                        }

                        moveit_visual_tools.publishAxis(pose_msg);
                        if (i == 0)
                        {
                            grasp_msg = pose_msg;
                            pre_grasp = pose_msg.position.z;
                            
                        }
                        else if (i == 1)
                        {
                            grasp = pose_msg.position.z;
                        }
                             
                    }
                    moveit_msgs::msg::Grasp vis_gr;
                    vis_gr.grasp_pose.header.frame_id = "base_footprint";
                    vis_gr.id = "anton";
                    vis_gr.grasp_pose.pose = grasp_msg;
                    std::vector<moveit_msgs::msg::Grasp> vis_grasps = {vis_gr};
                    

                    if (pre_grasp > grasp)
                    {
                        moveit_visual_tools.publishGrasps(vis_grasps, jmp);
                        moveit_visual_tools.trigger();
                        moveit_visual_tools.prompt("Next");
                    }
                    


                    //moveit_visual_tools.trigger();
                    moveit_visual_tools.deleteAllMarkers();
                    moveit_visual_tools.trigger();
                }
            }
        }    
    }//);
    //executor.spin();
    rclcpp::on_shutdown([&spinner]() {
        spinner.join(); // Zamykamy wątek spinnera
    });

    rclcpp::shutdown();
    return 0;
}
