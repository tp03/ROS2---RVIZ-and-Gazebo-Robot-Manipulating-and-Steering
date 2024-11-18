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

using moveit::planning_interface::MoveGroupInterface;
namespace rvt = rviz_visual_tools;

int main(int argc, char * argv[])
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("pick_place");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

    auto move_group_interface_arm = MoveGroupInterface(node, "arm");

    move_group_interface_arm.setMaxVelocityScalingFactor(0.5);  // Zwiększamy prędkość
    move_group_interface_arm.setMaxAccelerationScalingFactor(0.5);  // Zwiększamy przyspieszenie

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

        moveit_visual_tools.publishCuboid(pose, 0.07, 0.07, 0.07, rvt::GREEN);

        // Wyświetlenie markerów
        moveit_visual_tools.trigger();

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "green_cube_3";
        collision_object.header.frame_id = "base_footprint";  // Lub "base_link", zależnie od układu odniesienia

        // Definiowanie kształtu obiektu (sześcian)
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.07, 0.07, 0.07};  // Wymiary obiektu

        // Pozycja i orientacja obiektu
        geometry_msgs::msg::Pose cube_pose;
        cube_pose.position.x = pose.position.x;  // Współrzędne pozycji
        cube_pose.position.y = pose.position.y;
        cube_pose.position.z = pose.position.z;
        cube_pose.orientation.w = pose.orientation.w;

        // Dodanie kształtu i pozycji do obiektu
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(cube_pose);
        collision_object.operation = collision_object.ADD;

        // Dodanie obiektu do sceny
        planning_scene_interface.applyCollisionObject(collision_object);


        float end_effector_x = pose.position.x;
        float end_effector_y = pose.position.y;
        float end_effector_z = pose.position.z + 0.3;

        move_group_interface_arm.setEndEffectorLink("wrist_ft_link");
        //move_group_interface_arm.setPoseReferenceFrame("base_footprint");

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << -1.0, 0.0,   0.0,
                          0.0,    1.0, 0.0,
                          0.0,    0.0,   -1.0;

        // Konwersja macierzy rotacji na kwaternion
        Eigen::Quaterniond quaternion(rotation_matrix);

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = quaternion.w(); 
        target_pose.orientation.x = quaternion.x(); 
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();// Przyjęcie orientacji jako jednostkowej kwaternionu
        target_pose.position.x = end_effector_x;     // Pozycja w metrach
        target_pose.position.y = end_effector_y;     
        target_pose.position.z = end_effector_z;

        move_group_interface_arm.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan4;
        move_group_interface_arm.setPlanningTime(10.0);
        auto const success4 = static_cast<bool>(move_group_interface_arm.plan(plan4));

        if (success4)
        {
          move_group_interface_arm.execute(plan4);

          auto move_group_interface_gripper = MoveGroupInterface(node, "gripper");
            move_group_interface_gripper.setMaxVelocityScalingFactor(0.7);  // Zwiększamy prędkość
            move_group_interface_gripper.setMaxAccelerationScalingFactor(0.7); 
            move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.04,0.042}));
            moveit::planning_interface::MoveGroupInterface::Plan plan5;
            auto const success5 = static_cast<bool>(move_group_interface_gripper.plan(plan5));

            if (success5)
            {
            move_group_interface_gripper.execute(plan5);
            }

            target_pose.position.z = end_effector_z-0.08;
            move_group_interface_arm.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan6;
            auto const success6 = static_cast<bool>(move_group_interface_arm.plan(plan6));

            if (success6)
            {
            move_group_interface_arm.execute(plan6);
            }

            std::vector<std::string> touch_links = {"gripper_left_finger_link", "gripper_right_finger_link"};
            auto const connected = static_cast<bool>(move_group_interface_arm.attachObject("green_cube_3", "wrist_ft_link", touch_links));

            move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.03,0.03}));
            moveit::planning_interface::MoveGroupInterface::Plan plan7;
            auto const success7 = static_cast<bool>(move_group_interface_gripper.plan(plan7));

            if (connected)
            {
            if (success7)
            {
                move_group_interface_arm.execute(plan7);
            }
            }
            target_pose.position.z = target_pose.position.z + 0.05;
            move_group_interface_arm.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan8;
            auto const success8 = static_cast<bool>(move_group_interface_arm.plan(plan8));

            if (connected)
            {
            if (success8)
            {
                move_group_interface_arm.execute(plan8);
            }
            }

            target_pose.position.y = -target_pose.position.y;
            move_group_interface_arm.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan9;
            auto const success9 = static_cast<bool>(move_group_interface_arm.plan(plan9));
            if (success9)
            {
                move_group_interface_arm.execute(plan9);
            }

            target_pose.position.z = target_pose.position.z - 0.048;
            move_group_interface_arm.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan10;
            auto const success10 = static_cast<bool>(move_group_interface_arm.plan(plan10));
            if (success10)
            {
                move_group_interface_arm.execute(plan10);
            }

            move_group_interface_gripper.setJointValueTarget(std::vector<double>({0.044,0.044}));
            moveit::planning_interface::MoveGroupInterface::Plan plan11;
            auto const success11 = static_cast<bool>(move_group_interface_gripper.plan(plan11));
            if (success11)
            {
                move_group_interface_gripper.execute(plan11);
            }

            auto const detached = static_cast<bool>(move_group_interface_arm.detachObject("green_cube_3"));


            }
        else{
          RCLCPP_ERROR(node->get_logger(), "Object too far");
        }        
    }


    else {
      RCLCPP_ERROR(node->get_logger(), "Service call failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
