#! /usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    TimerAction, 
    DeclareLaunchArgument, 
    SetEnvironmentVariable, 
    IncludeLaunchDescription, 
    RegisterEventHandler,
    ExecuteProcess  # <--- AGGIUNTO
)
from launch.event_handlers import OnProcessExit, OnProcessStart 
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Environment: ros2_control plugin ---
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[
            '/opt/ros/humble/lib:',
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # --- Paths ---
    world_file = os.path.join(
        get_package_share_directory('ros2_fra2mo'),
        'worlds', 'warehouse.sdf'
    )
    
    pkg_kdl = get_package_share_directory('ros2_kdl_package')
    bridge_config_file = os.path.join(pkg_kdl, 'config', 'bridge_config.yaml')

    # Fra2mo description
    fra2mo_xacro = os.path.join(
        get_package_share_directory('ros2_fra2mo'),
        'urdf', 'fra2mo.urdf.xacro'
    )

    fra2mo_desc = {
        "robot_description": ParameterValue(
            Command(['xacro ', fra2mo_xacro]),
            value_type=str
        )
    }

    # --- IIWA Arguments ---
    iiwa_description_arg = DeclareLaunchArgument(
        'iiwa_description_file',
        default_value='iiwa.config.xacro'
    )

    config_file = LaunchConfiguration('iiwa_description_file')

    # --- IIWA Robot description ---
    xacro_path = PathJoinSubstitution([
        FindPackageShare('iiwa_description'),
        'config',
        config_file
    ])
   
    iiwa_pose_config = PathJoinSubstitution([
        FindPackageShare('ros2_fra2mo'),
        'config',
        'iiwa.pose.yaml'
    ])

    iiwa_desc = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_path,
                ' prefix:=iiwa_',
                ' parent:=world',
                ' use_sim:=true',
                ' namespace:=iiwa',
                ' command_interface:=position',
                ' base_frame_file:=', iiwa_pose_config
            ]),
            value_type=str
        )
    }

    # --- Robot state publishers ---
    fra2mo_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fra2mo',
        parameters=[fra2mo_desc],
        output='screen'
    )

    iiwa_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='iiwa',
        parameters=[iiwa_desc],
        remappings=[
            ('/robot_description', 'robot_description')
        ],
        output='screen'
    )

    # --- Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/iiwa_camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/iiwa_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/iiwa/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/iiwa/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
            '/gripper/state@std_msgs/msg/Bool@ignition.msgs.Boolean',
            '/gripper/detach@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/gripper/attach@std_msgs/msg/Empty@ignition.msgs.Empty'
        ],
        output='screen'
    )
    
    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    gz_args = DeclareLaunchArgument(
        'gz_args',
        default_value=world_file + ' -r',
        description='Gazebo world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/fra2mo/robot_description',
            '-name', 'fra2mo',
            '-x', '-9.2', '-y', '3.1', '-z', '0.15'
        ],
        output='screen'
    )
    
    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/iiwa/robot_description',
            '-name', 'iiwa',
            '-x', '8.0', '-y', '-4.0', '-z', '0.1'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace='iiwa',
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['iiwa_arm_controller'],
        namespace='iiwa',
        output='screen'
    )

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[joint_state_broadcaster, arm_controller]
        )
    )
    
    # --- Gestione Detach Pacco ---
    detach_package = ExecuteProcess(
       cmd=['ros2', 'topic', 'pub', '--once', '/gripper/detach', 'std_msgs/msg/Empty', '{}'],
       output='screen'
    )
    
    detach_handler = RegisterEventHandler(
       OnProcessStart(
           target_action=bridge,
           on_start=[
               TimerAction(
                   period=10.0,
                   actions=[detach_package]
               )
           ]
       )
    )

    static_tf_world_to_fra2mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'fra2mo/odom'],
        output='screen'
    )

    return LaunchDescription([
        set_plugin_path,
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=os.path.join(get_package_share_directory('ros2_fra2mo'), 'models') + ':' +
                  os.path.join(get_package_share_directory('iiwa_description')) + ':' +
                  os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        iiwa_description_arg,
        gz_args,
        bridge,
        odom_tf,
        static_tf_world_to_fra2mo,
        gazebo,
        fra2mo_rsp,
        spawn_fra2mo,
        iiwa_rsp,
        spawn_iiwa,
        controllers_after_spawn,
        detach_handler # <--- AGGIUNTO QUI per farlo attivare
    ])
