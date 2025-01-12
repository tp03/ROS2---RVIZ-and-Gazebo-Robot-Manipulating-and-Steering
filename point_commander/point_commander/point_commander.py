import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
from geometry_msgs.msg import Quaternion

class point_commander(Node):

    def __init__(self):
        super().__init__('point_commander')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('place', "Kitchen")

        self.place = self.get_parameter('place').get_parameter_value().string_value


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()


        if self.place == "Kitchen":

            goal_msg.pose.pose.position.x = -3.65
            goal_msg.pose.pose.position.y = -0.844
            goal_msg.pose.pose.position.z = 0.0
        
            goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, -0.26)

        elif self.place == "WC":

            goal_msg.pose.pose.position.x = -2.56
            goal_msg.pose.pose.position.y = -3.8
            goal_msg.pose.pose.position.z = 0.0
        
            goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, 1.42)

        elif self.place == "Wardrobe":

            goal_msg.pose.pose.position.x = 2.35
            goal_msg.pose.pose.position.y = 4.8
            goal_msg.pose.pose.position.z = 0.0
        
            goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, -1.595)

        elif self.place == "Bedroom":

            goal_msg.pose.pose.position.x = 4.13
            goal_msg.pose.pose.position.y = 0.52
            goal_msg.pose.pose.position.z = 0.0
        
            goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, -1.67)

        elif self.place == "Sofa":

            goal_msg.pose.pose.position.x = -3.48
            goal_msg.pose.pose.position.y = -7.23
            goal_msg.pose.pose.position.z = 0.0
        
            goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, 0.25)

        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

def main(args=None):
    rclpy.init(args=args)
    node = point_commander()

    node.send_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        