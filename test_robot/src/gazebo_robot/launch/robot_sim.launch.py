import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    os.environ["DISPLAY"] = ":1"
    pkg_share = get_package_share_directory('gazebo_robot')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    model_file = os.path.join(pkg_share, 'models', 'simple_robot.sdf')

    # Start Gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty_world/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '1000',
             '--req', f'sdf_filename: "{model_file}"'],
        output='screen'
    )

    # Bridge ROS2 topics to Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_sim,
                on_exit=[spawn_robot],
            )
        ),
        bridge,
    ])
