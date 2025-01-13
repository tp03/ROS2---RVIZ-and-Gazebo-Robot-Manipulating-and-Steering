import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint   
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import math

class projekt2(Node):

    def __init__(self):
        super().__init__('projekt2')

        self.basic_navigator = BasicNavigator()

        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.velocity_subscriber = self.create_subscription(Twist, '/cmd_vel_nav', self.velocity_callback, 10)

        self.current_velocity = 0.0

    def velocity_callback(self, msg):

        self.current_velocity = msg.angular.z
        if abs(self.current_velocity) < 0.1:
            self.move_head(0.0)
        else:
            angle = self.current_velocity*math.pi/3
            if self.current_velocity > 0.0:
                self.move_head(angle)
            else:
                self.move_head(-angle)

    def move_head(self, head_1_angle, time_from_start=1):

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [head_1_angle, 0.0]
        point.time_from_start.sec = int(time_from_start)

        trajectory_msg.points = [point]
        self.head_publisher.publish(trajectory_msg)

    def make_pose_stamped(self, x, y, yaw):

        rclpy.spin_once(self, timeout_sec=0.1)

        pose = PoseStamped()
        pose.header.frame_id = "map"  
        pose.header.stamp = rclpy.time.Time().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  

        quaternion = self.euler_to_quaternion(0.0, 0.0, yaw)  
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


    def navigate_with_head_control(self, points):
        self.basic_navigator.lifecycleStartup()

        for idx, point in enumerate(points):
            x, y, yaw = point

            goal_pose = self.make_pose_stamped(x, y, yaw)
            self.get_logger().info(f'Sending navigation goal to point {idx + 1}/{len(points)}: x={x}, y={y}, yaw={yaw}')

            self.basic_navigator.goToPose(goal_pose)

            while not self.basic_navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)
                #self.get_logger().info("Movin")
                continue
            self.move_head(0.0)

        self.basic_navigator.lifecycleShutdown()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    node = projekt2()
    rclpy.spin_once(node, timeout_sec=0.1)

    points = [
        (-3.65, -0.844, -0.26),
        (4.13, 0.52, -1.67),
        (-2.56, -3.8, 1.42),
        (2.35, 4.8, -1.595),
    ]

    try:
        node.navigate_with_head_control(points)        
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
