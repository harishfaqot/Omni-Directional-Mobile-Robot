import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('robotpadi_description')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'robotpadi.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    rviz_config = os.path.join(pkg_ros_gz_rbot, 'config', 'nav_goal.rviz')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 --render-engine ogre empty.sdf"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "robotpadi",
                "-allow_renaming", "false",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    load_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    load_steering_controller = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['steering_controller'],
            output='screen',
        )]
    )

    load_wheel_controller = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_controller'],
            output='screen',
        )]
    )

    # Static TF: map -> odom (identity, since we have no SLAM/localization)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Odometry publisher: publishes odom->base_link from Gazebo ground truth pose
    # We use ros_gz_bridge to get /model/robotpadi/pose, then publish odom TF from it
    odom_tf_publisher = TimerAction(
        period=12.0,
        actions=[Node(
            package='robotpadi_controller',
            executable='nav_goal_driver',
            name='nav_goal_driver',
            output='screen',
        )]
    )

    # Simple odom->base_link TF from Gazebo pose bridge
    gz_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_pose_bridge',
        arguments=[
            '/model/robotpadi/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/robotpadi/pose', '/tf'),
        ],
        output='screen'
    )

    # RViz
    rviz = TimerAction(
        period=8.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen',
        )]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        gz_odom_bridge,
        robot_state_publisher,
        map_to_odom_tf,
        load_joint_state_broadcaster,
        load_steering_controller,
        load_wheel_controller,
        odom_tf_publisher,
        rviz,
    ])
