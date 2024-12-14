from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'projekt1',
            namespace= 'projekt1',
            executable='projekt1',
            parameters=[{
            	"a": LaunchConfiguration('a'),
            	},
            	{
            	"side": LaunchConfiguration('side'),
                },
                {
                "n": LaunchConfiguration('n'),
                }                
            ]
        )       
    ])