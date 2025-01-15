import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  
from nav2_msgs.action import FollowWaypoints 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, ActionClient
from custom_action_servers.action import NavigatePoints
import math
import time
from action_msgs.msg import GoalStatus

class projekt2(Node):

    def __init__(self):
        super().__init__('projekt2')

        self.basic_navigator = BasicNavigator()

        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.velocity_subscriber = self.create_subscription(Twist, '/cmd_vel_nav', self.velocity_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.action_server = ActionServer(self, NavigatePoints, 'navigate_points', self.server_callback)
        self.action_client = ActionClient(self, NavigatePoints, 'navigate_points')

        self.current_velocity = 0.0

        self.previous_pose = []
        self.current_pose = [] 

        self.distance_made = 0.0
    
    def odom_callback(self, msg):

        self.previous_pose = self.current_pose
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.previous_pose = self.current_pose if self.previous_pose == [] else self.previous_pose

        dx = self.current_pose[0] - self.previous_pose[0]
        dy = self.current_pose[1] - self.previous_pose[1]
        self.distance_made += math.sqrt(dx**2 + dy**2)
    
    def velocity_callback(self, msg):

        self.current_velocity = msg.angular.z
        if abs(self.current_velocity) < 0.01:
            self.move_head(0.0)
        else:
            angle = self.current_velocity*math.pi/3
            self.move_head(angle)

    def wait_for_initial_pose(self, timeout=5.0):
        start_time = self.get_clock().now()
        while self.previous_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() - start_time > timeout:
                return False
        return True


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
        pose.header.stamp = self.get_clock().now().to_msg()

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
    
    def calculate_path_length(self, path_points):
        length = 0.0
        for i in range(len(path_points) - 1):
            x1, y1 = path_points[i].pose.position.x, path_points[i].pose.position.y
            x2, y2 = path_points[i + 1].pose.position.x, path_points[i + 1].pose.position.y
            segment_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            length += segment_length
        return length

    def isNavComplete(self):
        if not self.result_future:
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                return True
        else:
            return False
        return True
        
    def followWaypoints(self, poses):

        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            pass

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def server_callback(self, goal_handle):

        self.wait_for_initial_pose()

        self.get_logger().info('Nawiązano połączenie!')

        rclpy.spin_once(self, timeout_sec=0.1)

        feedback = NavigatePoints.Feedback()
        points = goal_handle.request.waypoints
        path_points = Path()
        begin_p = self.make_pose_stamped(self.current_pose[0], self.current_pose[1], 0.0)
        path_points = self.basic_navigator.getPathThroughPoses(begin_p, points)
        full_distance = self.calculate_path_length(path_points.poses)

        self.followWaypoints(points)

        while not self.isNavComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback.progress = self.distance_made/full_distance*100
            goal_handle.publish_feedback(feedback)
         
        goal_handle.succeed()
        self.get_logger().info("Zakończono") 
        result = NavigatePoints.Result()
        result.success = True

        return NavigatePoints.Result(success=True)

    def send_server_goal(self, points):

        goal_msg = NavigatePoints.Goal()

        prepared_points = []

        for point in points:

            prepared_points.append(self.make_pose_stamped(point[0], point[1], point[2]))      

        goal_msg.waypoints = prepared_points

        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Żądanie odrzucone.')
            return

        self.get_logger().info('Żądanie zaakceptowane.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Wszystkie punkty osiągnięte.')
        else:
            self.get_logger().info('Nie udało się osiągnąć wszystkich punktów.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Wykonano procent ścieżki: {round(feedback.progress)}%')


def main(args=None):
    rclpy.init(args=args)

    node = projekt2()

    points = [
        (-3.65, -0.844, -0.26),
        (4.13, 0.52, -1.67),
        (-2.56, -3.8, 1.42),
        (2.35, 4.8, -1.595),
    ]

    try:
        time.sleep(3)
        node.send_server_goal(points)
        rclpy.spin(node)        
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
