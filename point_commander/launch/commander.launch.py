from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'point_commander',
            namespace= '',
            executable='point_commander',
            parameters=[{
            	"place": LaunchConfiguration('place'),
            	},           
            ]
        )       
    ])