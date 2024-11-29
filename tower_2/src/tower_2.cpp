#include <memory>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
// #include <geometry_msgs/Pose.h>
#include <thread>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;
namespace rvt = rviz_visual_tools;
using namespace Eigen;


geometry_msgs::msg::Pose get_cube_pose(std::string cube_name, auto client, std::vector<moveit_msgs::msg::CollisionObject> collision_objects)
{

    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = cube_name.c_str(); // Nazwa obiektu w symulatorze
    request->reference_frame = "base_footprint";

    // Wysyłamy zapytanie i czekamy na odpowiedź
    auto result_future = client->async_send_request(request);

    geometry_msgs::msg::Pose pose;

    if (result_future.wait_for(std::chrono::seconds(3)) == std::future_status::ready)
    {
        pose = result_future.get()->state.pose;

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = cube_name.c_str();
        collision_object.header.frame_id = "base_footprint"; // Lub "base_link", zależnie od układu odniesienia
        RCLCPP_INFO(rclcpp::get_logger("get_cube_pose"), "here2");

        // Definiowanie kształtu obiektu (sześcian)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.07, 0.07, 0.07}; // Wymiary obiektu
        RCLCPP_INFO(rclcpp::get_logger("get_cube_pose"), "here3");


        // Dodanie kształtu i pozycji do obiektu
        collision_object.primitives.push_back(primitive);
        RCLCPP_INFO(rclcpp::get_logger("get_cube_pose"), "here4");

        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        RCLCPP_INFO(rclcpp::get_logger("get_cube_pose"), "here5");


        // Dodanie obiektu do sceny
        collision_objects.push_back(collision_object);
        RCLCPP_INFO(rclcpp::get_logger("get_cube_pose"), "here6");

    }
    return pose;
}

std::vector<std::vector<geometry_msgs::msg::Pose>> get_grasps(geometry_msgs::msg::Pose pose)
{

        geometry_msgs::msg::Pose saved_pose = pose;
        
        std::vector<geometry_msgs::msg::Pose> pre_grasps = {};
        std::vector<geometry_msgs::msg::Pose> grasps = {};

        Eigen::Isometry3d nothing(AngleAxisd(0.0, Vector3d::UnitX()));
        Eigen::Isometry3d rot_z_90(AngleAxisd(90.0 / 180.0 * M_PI, Vector3d::UnitZ()));
        Eigen::Isometry3d rot_y_180(AngleAxisd(M_PI, Vector3d::UnitY()));
        Eigen::Isometry3d rot_x_30(AngleAxisd(30.0 / 180.0 * M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_minus_30(AngleAxisd(-30.0 / 180.0 * M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_90(AngleAxisd(90.0 / 180.0 * M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_x_minus_90(AngleAxisd(-90.0 / 180.0 * M_PI, Vector3d::UnitX()));
        Eigen::Isometry3d rot_y_90(AngleAxisd(90.0 / 180.0 * M_PI, Vector3d::UnitY()));
        Eigen::Isometry3d rot_y_minus_90(AngleAxisd(-90.0 / 180.0 * M_PI, Vector3d::UnitY()));

        for (int w = 0; w < 5; w++)
        {

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
            for (int j = 0; j < 4; j++)
            {
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
                    bambi = rot_z_90 * rot_z_90;
                }
                else
                {
                    bambi = rot_z_90 * rot_z_90 * rot_z_90;
                }

                for (int m = 0; m < 3; m++)
                {

                    float pre_grasp = 0.0;
                    float grasp = 0.0;
                    geometry_msgs::msg::Pose pre_grasp_pose;
                    geometry_msgs::msg::Pose grasp_pose;

                    for (int i = 0; i < 3; i++)
                    {

                        geometry_msgs::msg::Pose pose_msg;
                        if (i == 0)
                        {
                            if (m == 0)
                            {
                                float _x = saved_pose.position.x;
                                float _z = saved_pose.position.z + 0.3;
                                float _y = saved_pose.position.y;
                                if (w == 0)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 1)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.3, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 2)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.3, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 3)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.3, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 4)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.3, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                            }
                            else
                            {
                                float _z = saved_pose.position.z + 0.26;
                                if (m == 1)
                                {
                                    if (j == 0 || j == 2)
                                    {
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y + 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.26, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.26, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y + 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y + 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _y = saved_pose.position.y - 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.26, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.26, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y - 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y - 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
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
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.15, saved_pose.position.y - 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.15, saved_pose.position.y + 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _x = saved_pose.position.x - 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.15, saved_pose.position.y - 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.15, saved_pose.position.y + 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                                else if (m == 2)
                                {
                                    if (j == 0 || j == 2)
                                    {
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y - 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.26, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.26, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y - 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y - 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _y = saved_pose.position.y + 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.26, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.26, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y + 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y + 0.15, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
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
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.15, saved_pose.position.y - 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.15, saved_pose.position.y + 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _x = saved_pose.position.x + 0.15;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.15, saved_pose.position.y - 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.15, saved_pose.position.y + 0.26, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.26, saved_pose.position.y, saved_pose.position.z - 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.26, saved_pose.position.y, saved_pose.position.z + 0.15));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else if (i == 1)
                        {
                            if (m == 0)
                            {
                                float _x = saved_pose.position.x;
                                float _z = saved_pose.position.z + 0.22;
                                float _y = saved_pose.position.y;

                                if (w == 0)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 1)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.22, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 2)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.22, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 3)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.22, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                                if (w == 4)
                                {
                                    Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.22, saved_pose.position.y, saved_pose.position.z));
                                    Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi;
                                    pose_msg = tf2::toMsg(skolim);
                                }
                            }
                            else
                            {
                                float _z = saved_pose.position.z + 0.19;
                                if (m == 1)
                                {
                                    if (j == 0 || j == 2)
                                    {
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y + 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.19, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.19, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y + 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y + 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _y = saved_pose.position.y - 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.19, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.19, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y - 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y - 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
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
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.11, saved_pose.position.y - 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.11, saved_pose.position.y + 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _x = saved_pose.position.x - 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.11, saved_pose.position.y - 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.11, saved_pose.position.y + 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                                else if (m == 2)
                                {
                                    if (j == 0 || j == 2)
                                    {
                                        float _x = saved_pose.position.x;
                                        if (j == 0)
                                        {
                                            float _y = saved_pose.position.y - 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.19, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.19, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y - 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y - 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _y = saved_pose.position.y + 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y - 0.19, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x, saved_pose.position.y + 0.19, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y + 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y + 0.11, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
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
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.11, saved_pose.position.y - 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.11, saved_pose.position.y + 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                        else
                                        {
                                            float _x = saved_pose.position.x + 0.11;
                                            if (w == 0)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(_x, _y, _z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 1)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.11, saved_pose.position.y - 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 2)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.11, saved_pose.position.y + 0.19, saved_pose.position.z));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 3)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x + 0.19, saved_pose.position.y, saved_pose.position.z - 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                            if (w == 4)
                                            {
                                                Eigen::Isometry3d tr(Eigen::Translation3d(saved_pose.position.x - 0.19, saved_pose.position.y, saved_pose.position.z + 0.11));
                                                Eigen::Isometry3d skolim = tr * which_side * rot_y_180 * bambi * rot_x_minus_30 * rot_z_90;
                                                pose_msg = tf2::toMsg(skolim);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            pose_msg.position.x = saved_pose.position.x;
                            pose_msg.position.y = saved_pose.position.y;
                            pose_msg.position.z = saved_pose.position.z;

                            pose_msg.orientation.x = saved_pose.orientation.x;
                            pose_msg.orientation.y = saved_pose.orientation.y;
                            pose_msg.orientation.z = saved_pose.orientation.z;
                            pose_msg.orientation.w = saved_pose.orientation.w;
                        }

                        if (i == 0)
                        {
                            pre_grasp = pose_msg.position.z;
                            pre_grasp_pose = pose_msg;
                        }
                        else if (i == 1)
                        {
                            grasp = pose_msg.position.z;
                            grasp_pose = pose_msg;
                        }
                    }

                    if (pre_grasp > grasp)
                    {
                        grasps.push_back(grasp_pose);
                        pre_grasps.push_back(pre_grasp_pose);
                    }
                }
            }

            std::vector<std::vector<geometry_msgs::msg::Pose>> vectors = {};
            vectors.push_back(pre_grasps);
            vectors.push_back(grasps);
            return vectors;
        }
}


int main(int argc, char *argv[])
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("tower_2");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]()
                               { executor.spin(); });

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Tworzymy klienta do serwisu /get_entity_state
    auto client = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");

    // Sprawdzamy, czy serwis jest dostępny
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupt signal received, shutting down...");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /get_entity_state to be available...");
    }

    auto move_group_interface_arm = MoveGroupInterface(node, "arm");

    move_group_interface_arm.setMaxVelocityScalingFactor(0.5);     // Zwiększamy prędkość
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.5); // Zwiększamy przyspieszenie

    move_group_interface_arm.setJointValueTarget(std::vector<double>{0.42, -1.26, -0.42, 2.13, -1.5, 1.4, 0.0});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const success = static_cast<bool>(move_group_interface_arm.plan(plan));

    if (success)
    {
        move_group_interface_arm.execute(plan);
    }

    move_group_interface_arm.setJointValueTarget(std::vector<double>{0.42, -1.26, -2.69, 2.13, -1.5, 1.4, 0.0});
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    auto const success2 = static_cast<bool>(move_group_interface_arm.plan(plan2));

    if (success2)
    {
        move_group_interface_arm.execute(plan2);
    }

    move_group_interface_arm.setJointValueTarget(std::vector<double>{1.58, 0.8, 0.24, 0.1, -1.47, 0.75, 0.0});
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    auto const success3 = static_cast<bool>(move_group_interface_arm.plan(plan3));

    if (success3)
    {
        move_group_interface_arm.execute(plan3);
    }

    // Tworzymy obiekt MoveItVisualTools do wyświetlania w RViz
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_footprint", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface_arm.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {};

    geometry_msgs::msg::Pose cube3_pose = get_cube_pose("green_cube_3", client, collision_objects);
    geometry_msgs::msg::Pose cube2_pose = get_cube_pose("green_cube_2", client, collision_objects);

    RCLCPP_INFO(node->get_logger(), "here");

    planning_scene_interface.applyCollisionObjects(collision_objects);

    RCLCPP_INFO(node->get_logger(), "here2");

    std::vector<std::vector<geometry_msgs::msg::Pose>> cube2_vectors = get_grasps(cube2_pose);
    std::vector<geometry_msgs::msg::Pose> cube2_pre_grasps = cube2_vectors[0];
    std::vector<geometry_msgs::msg::Pose> cube2_grasps = cube2_vectors[1];

    for (int i = 1; i < cube2_pre_grasps.size(); i++)
    {

        move_group_interface_arm.setEndEffectorLink("wrist_ft_link");

        geometry_msgs::msg::Pose target_pose = cube2_pre_grasps[i];

        move_group_interface_arm.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan4;
        move_group_interface_arm.setPlanningTime(10.0);
        auto const success4 = static_cast<bool>(move_group_interface_arm.plan(plan4));

        if (success4)
        {
            move_group_interface_arm.execute(plan4);
            auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");
            move_group_interface_gripper.setMaxVelocityScalingFactor(0.7); // Zwiększamy prędkość
            move_group_interface_gripper.setMaxAccelerationScalingFactor(0.7);
            move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.04, 0.042}));
            moveit::planning_interface::MoveGroupInterface::Plan plan5;
            auto const success5 = static_cast<bool>(move_group_interface_gripper.plan(plan5));

            if (success5)
            {
                target_pose = cube2_grasps[i];
                move_group_interface_gripper.execute(plan5);
                move_group_interface_arm.setPoseTarget(target_pose);
                moveit::planning_interface::MoveGroupInterface::Plan plan6;
                auto const success6 = static_cast<bool>(move_group_interface_arm.plan(plan6));

                if (success6)
                {
                    move_group_interface_arm.execute(plan6);
                    std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};
                    auto const connected = static_cast<bool>(move_group_interface_arm.attachObject("green_cube_2", "wrist_ft_link", touch_links));

                    move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.0275, 0.0275}));
                    moveit::planning_interface::MoveGroupInterface::Plan plan7;
                    auto const success7 = static_cast<bool>(move_group_interface_gripper.plan(plan7));

                    if (connected)
                    {
                        if (success7)
                        {
                            move_group_interface_arm.execute(plan7);
                        }
                    }

                    std::vector<std::vector<geometry_msgs::msg::Pose>> cube3_vectors = get_grasps(cube3_pose);
                    std::vector<geometry_msgs::msg::Pose> cube3_pre_grasps = cube3_vectors[0];
                    std::vector<geometry_msgs::msg::Pose> cube3_grasps = cube3_vectors[1];

                    for (int j = 0; j < cube3_grasps.size(); j++)
                    {
                        RCLCPP_INFO(node->get_logger(), "%d", j);
                        geometry_msgs::msg::Pose old_pose = cube2_grasps[i];
                        target_pose = cube3_grasps[j];
                        if (std::abs(target_pose.position.z - old_pose.position.z) < 0.005)
                        {
                            RCLCPP_INFO(node->get_logger(), "Good, 1: %.3f, 2: %.3f", target_pose.position.z, old_pose.position.z);
                            target_pose.position.z = target_pose.position.z + 0.23;
                            move_group_interface_arm.setPoseTarget(target_pose);
                            moveit::planning_interface::MoveGroupInterface::Plan plan9;
                            auto const success9 = static_cast<bool>(move_group_interface_arm.plan(plan9));
                            if (success9)
                            {
                                move_group_interface_arm.execute(plan9);
                                target_pose.position.z = target_pose.position.z - 0.1;
                                move_group_interface_arm.setPoseTarget(target_pose);
                                moveit::planning_interface::MoveGroupInterface::Plan plan10;
                                auto const success10 = static_cast<bool>(move_group_interface_arm.plan(plan10));
                                if (success10)
                                {
                                    move_group_interface_arm.execute(plan10);
                                }

                                move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044, 0.044}));
                                moveit::planning_interface::MoveGroupInterface::Plan plan11;
                                auto const success11 = static_cast<bool>(move_group_interface_gripper.plan(plan11));
                                if (success11)
                                {
                                    move_group_interface_gripper.execute(plan11);
                                }

                                auto const detached = static_cast<bool>(move_group_interface_arm.detachObject("green_cube_2"));
                                break;
                            }

                        }
                        else
                        {
                            RCLCPP_INFO(node->get_logger(), "Bad, 1: %.3f, 2: %.3f", target_pose.position.z, old_pose.position.z);
                        }
                        
                    }
                    break;
                }
            }
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Bad");
        }
    }
    rclcpp::shutdown();
    return 0;
}
