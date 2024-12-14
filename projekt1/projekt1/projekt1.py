import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from math import atan2, sqrt, pow
from numpy import pi

class projekt1(Node):

    def __init__(self):
        super().__init__('projekt1')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.subscriber = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)

        self.declare_parameter('a', 3)
        self.declare_parameter('side', "rigth")
        self.declare_parameter('n', 1)

        self.a = self.get_parameter('a').get_parameter_value().double_value
        self.side = self.get_parameter('side').get_parameter_value().string_value
        self.n = self.get_parameter('n').get_parameter_value().integer_value

        self.current_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.x_perspective = 0.0  
        self.y_perspective = 0.0 
        self.theta_perspective = 0.0 
        self.desired_theta = pi/2

        self.i = 0

    def odom_callback(self, msg):
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)

        if atan2(siny_cosp, cosy_cosp) < 0.0:
            # self.i += 1
            # if self.i == 1:
            #     self.theta_perspective+= pi/2
            self.current_position['theta'] = 2*pi - abs(atan2(siny_cosp, cosy_cosp))
        elif atan2(siny_cosp, cosy_cosp) > 6.28:
            # self.i += 1
            # if self.i == 1:
            #     self.theta_perspective+= pi/2
            self.current_position['theta'] = 2*pi + abs(atan2(siny_cosp, cosy_cosp))
        else:
            self.current_position['theta'] = atan2(siny_cosp, cosy_cosp)


    def move(self):

        time.sleep(3)
        rclpy.spin_once(self, timeout_sec=0.1)

        for i in range(4):

            self.theta_perspective = self.current_position['theta']
            angular_distance = abs(self.current_position['theta']- self.theta_perspective)
            er = self.desired_theta - angular_distance
            while er > 0.011:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(f"Odległość: {angular_distance}, kąt: {self.current_position['theta']}, err: {er}")
                self.send_velocity(0.0, 0.3)
                angular_distance = abs(self.current_position['theta'] - self.theta_perspective)
                er = self.desired_theta - angular_distance

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

                self.get_logger().info(f"Odległość: {distance}, błąd: {err}")

            self.stop()



    def send_velocity(self, linear_vel, angular_vel):
        """Publishes a Twist message with given linear and angular velocities."""
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.publisher.publish(cmd_vel)

    def stop(self):
        """Stops the robot by sending zero velocities."""
        self.send_velocity(0.0, 0.0)

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



        