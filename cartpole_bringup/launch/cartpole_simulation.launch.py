import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


bringup_dir = get_package_share_directory('cartpole_bringup')
pybullet_ros_dir = get_package_share_directory('pybullet_ros')


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    control = LaunchConfiguration('control')
    control_mode = LaunchConfiguration('control_mode')
    remappings = [('/slider_cart_effort_controller/command', '/cartpole/acc_d')]

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='cartpole',
        description='Top-level namespace')

    control_arg = DeclareLaunchArgument(
        'control',
        default_value='True',
        description='Whether incorporate a controller')

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='energy',
        description='Control mode [manual | energy | mpc]')

    bringup_action_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'cartpole_controller.launch.py')),
            condition=IfCondition(control),
            launch_arguments={'namespace': namespace,
                              'control_mode': control_mode,}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pybullet_ros_dir, 'cartpole_pybullet_ros.launch.py'))
        ),
    ])

    launch_description = LaunchDescription()
    launch_description.add_action(namespace_arg)
    launch_description.add_action(control_arg)
    launch_description.add_action(control_mode_arg)
    launch_description.add_action(bringup_action_group)

    return launch_description