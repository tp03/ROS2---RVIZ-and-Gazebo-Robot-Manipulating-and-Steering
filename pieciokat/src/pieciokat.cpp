#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>  
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;


void send_velocity(float linear_vel, float radial_vel, auto publisher)
{

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = radial_vel;
    

    publisher->publish(cmd_vel); 
}

void stop(auto publisher)
{

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    

    publisher->publish(cmd_vel); 

}


int main(int argc, char * argv[])
{
    // Inicjalizacja ROS2
    rclcpp::init(argc, argv);
    // Tworzymy węzeł ROS
    auto node = rclcpp::Node::make_shared("pieciokat");

    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
    for(int i = 0; i<5; i++)
    {
        auto timer_callback = [&publisher]() {
            send_velocity(0.0, 0.5, publisher);
        };
        auto timer = node->create_wall_timer(1000ms, timer_callback);
        
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && std::chrono::steady_clock::now() - start < 12.5s) {
            rclcpp::spin_some(node);
        }
        stop(publisher);

        timer.reset(); 
        auto timer_callback2 = [&publisher]() {
            send_velocity(0.5, 0.0, publisher);
        };
        auto timer2 = node->create_wall_timer(100ms, timer_callback2);
        
        auto start2 = std::chrono::steady_clock::now();
        while (rclcpp::ok() && std::chrono::steady_clock::now() - start2 < 6s) {
            rclcpp::spin_some(node);
        }
        stop(publisher);
        timer2.reset(); 
    }
    // const auto start = std::chrono::high_resolution_clock::now();
    // std::this_thread::sleep_for(5000ms);
    // const auto end = std::chrono::high_resolution_clock::now();
    // const std::chrono::duration<double, std::milli> elapsed = end - start;

    // stop(publisher);



    // //rclcpp::spin(node);
    // // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
