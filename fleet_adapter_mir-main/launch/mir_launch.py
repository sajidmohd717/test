from launch import LaunchDescription
from launch_ros.actions import Node
import os

config_folder = os.path.realpath(os.path.join(os.path.dirname(__file__),'../../../..','src/fleet_adapter_mir/configs'))
config = os.path.join(config_folder,'mir_config.yaml')
nav = os.path.join(config_folder,'nav_graph.yaml')
api = os.path.join(config_folder,'API_config.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fleet_adapter_mir',
            executable='fleet_adapter_mir',
            arguments=['-c', config,'-n',nav,'-a',api]
        )
    ])
