import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from math import atan2, sqrt, pow, fabs, pi
from numpy import pi
import matplotlib.pyplot as plt 

class projekt1(Node):

    def __init__(self):
        super().__init__('projekt1')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.ground_truth_subscriber = self.create_subscription(Odometry, '/ground_truth_odom', self.ground_truth_callback, 10)

        self.declare_parameter('a', 3)
        self.declare_parameter('side', "rigth")
        self.declare_parameter('n', 1)

        self.a = self.get_parameter('a').get_parameter_value().integer_value
        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.n = self.get_parameter('n').get_parameter_value().integer_value

        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.ground_truth_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.initial_position = {'x': 0.0, 'y': 0.0}
        self.fixed_degree = 0.0
        self.initial_orientation = 0.0
        self.position_squared_sum = 0.0
        self.orientation_squared_sum = 0.0
        self.num_samples = 0

        self.temporary_errors = []
        self.cumulative_errors = []
        self.start_time = time.time()
        self.last_report_time = 0.0
        self.i = 0

        self.position_error_sum = 0.0
        self.orientation_error_sum = 0.0

        self.x_perspective = 0.0
        self.y_perspective = 0.0



    def odom_callback(self, msg):
        """Callback from /mobile_base_controller/odom."""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y
        self.current_position['theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        #self.fixed_degree = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if self.quaternion_to_yaw(msg.pose.pose.orientation) < 0.0:

            self.i = 1

        if self.i == 1:
            if self.quaternion_to_yaw(msg.pose.pose.orientation) > 0.0:
                self.fixed_degree = 2*pi + abs(self.quaternion_to_yaw(msg.pose.pose.orientation))                            
            else:
                self.fixed_degree = 2*pi - abs(self.quaternion_to_yaw(msg.pose.pose.orientation))
        else:
            self.fixed_degree = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def ground_truth_callback(self, msg):
        """Callback from /ground_truth_odom."""
        self.ground_truth_position['x'] = msg.pose.pose.position.x
        self.ground_truth_position['y'] = msg.pose.pose.position.y
        self.ground_truth_position['theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.calculate_errors()
        self.check_temporary_errors()


    def quaternion_to_yaw(self, orientation_q):
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        return atan2(siny_cosp, cosy_cosp)

    def calculate_errors(self):
        dx = self.current_position['x'] - self.ground_truth_position['x']
        dy = self.current_position['y'] - self.ground_truth_position['y']
        position_error_squared = (dx**2 + dy**2)
        orientation_error_squared = fabs(self.current_position['theta'] - self.ground_truth_position['theta'])**2

        self.position_squared_sum += position_error_squared
        self.orientation_squared_sum += orientation_error_squared
        self.num_samples += 1

    def check_temporary_errors(self):
        # current_time = time.time() - self.start_time
        mean_position_error = sqrt(self.position_squared_sum / self.num_samples)
        mean_orientation_error = sqrt(self.orientation_squared_sum / self.num_samples)

        self.temporary_errors.append(( mean_position_error, mean_orientation_error))
        # self.last_report_time = current_time

    def move(self):
        self.get_logger().info("Starting robot movement...")
        time.sleep(3)
        rclpy.spin_once(self, timeout_sec=0.1)
        for loop_index in range(self.n):

            for side in range(4):
                self.theta_perspective = self.fixed_degree
                angular_distance = abs(self.fixed_degree- self.theta_perspective)
                er = pi/2 - angular_distance
                while er > 0.011:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.side == "left":
                        self.send_velocity(0.0, 0.3)
                    elif self.side == "rigth":
                        self.send_velocity(0.0, -0.3)
                    angular_distance = abs(self.fixed_degree - self.theta_perspective)
                    er = pi/2 - angular_distance

                self.i = 0
                self.stop()

                self.x_perspective = self.current_position['x']
                self.y_perspective = self.current_position['y']

                dx = self.current_position['x'] - self.x_perspective
                dy = self.current_position['y'] - self.y_perspective

                distance = sqrt(dx**2 + dy**2)

                err = self.a - distance

                while err > 0.1:

                    rclpy.spin_once(self, timeout_sec=0.1)
                    self.send_velocity(0.2, 0.0)

                    dx = self.current_position['x'] - self.x_perspective
                    dy = self.current_position['y'] - self.y_perspective

                    distance = sqrt(dx**2 + dy**2)

                    err = self.a - distance

                self.stop()


            mean_position_error = sqrt(self.position_squared_sum / self.num_samples)
            mean_orientation_error = sqrt(self.orientation_squared_sum / self.num_samples)
            self.cumulative_errors.append((loop_index + 1, mean_position_error, mean_orientation_error))

            self.get_logger().info(
                f"Loop {loop_index + 1} Complete - Cumulative Errors: Position: {mean_position_error:.6f}, "
                f"Orientation: {mean_orientation_error:.6f}"
            )

        self.stop()
        self.generate_report()


    def send_velocity(self, linear_vel, angular_vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.publisher.publish(cmd_vel)

    def stop(self):
        self.send_velocity(0.0, 0.0)
    
    def generate_report(self):
        self.get_logger().info("\n===================== Temporary Errors =====================")
        self.get_logger().info(f"{'Second':<10} {'Position':<10} {'Orientation':<10}")
        for pos, orient in self.temporary_errors[::10]:
            self.get_logger().info(f" {pos:<10.6f} {orient:<10.6f}")

        self.get_logger().info("\n===================== Cumulative Errors =====================")
        self.get_logger().info(f"{'Loop':<10} {'Position':<10} {'Orientation':<10}")
        for loop, pos, orient in self.cumulative_errors:
            self.get_logger().info(f"{loop:<10} {pos:<10.6f} {orient:<10.6f}")
        
        temp_pos, temp_orient = zip(*self.temporary_errors)
        loops, cum_pos, cum_orient = zip(*self.cumulative_errors)

        plt.figure(figsize=(15, 5))
        plt.subplot(1, 3, 1)
        plt.plot(temp_pos, 'o-',label='Position Error')
        plt.xlabel('Iteration')
        plt.ylabel('Error')
        plt.title('Temporary Errors')
        plt.legend()

        plt.subplot(1, 3, 2)
        plt.plot(temp_orient, 'o-',label='Orientation Error')
        plt.xlabel('Iteration')
        plt.ylabel('Error')
        plt.title('Temporary Errors')
        plt.legend()

        plt.subplot(1, 3, 3)
        plt.plot(loops, cum_pos, 'o-', label='Position Error')
        plt.plot(loops, cum_orient, 'o-', label='Orientation Error')
        plt.xlabel('Loop')
        plt.ylabel('Error')
        plt.title('Cumulative Errors')
        plt.legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = projekt1()

    try:
        node.move()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



        