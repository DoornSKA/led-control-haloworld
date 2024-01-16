from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    print(os.getcwd())
    return LaunchDescription([
        # Node(
        #     package='led_state',
        #     executable='publisher',
        #     name='pub'
        # ),
        Node(
            package='led_state',
            executable='listener',
            name='sub'
        ),
        Node(
            package='led_state',
            executable='leds',
            name='state',
        )
    ])