import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Percorso del file dei parametri
    config_params = PathJoinSubstitution([
        FindPackageShare('ros2_kdl_package'),
        'config',
        'kdl_params.yaml'
    ])

    # --- ARGOMENTI ---
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface', 
        default_value='position',
        description='Select controller: position, velocity or effort'
    )
    
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', 
        default_value='velocity_ctrl_null',
        description='Select velocity controller: velocity_ctrl, velocity_ctrl_null or vision'
    )

    # --- NODO BRIDGE (IGNITION -> ROS 2) ---
    # Questo nodo traduce i topic della camera per ROS
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # --- NODO ARUCO DETECTION ---
    # Questo nodo analizza le immagini e pubblica /aruco_single/pose
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rect': True,
            'marker_size': 0.08,         # Dimensione del tag in metri
            'marker_id': 201,            # ID del tag (assicurati coincida con Gazebo)
            'reference_frame': 'iiwa_iiwa_base',
            'camera_frame': 'iiwa/iiwa_link_7/iiwa_camera',
            'marker_frame': 'aruco_marker_frame',
            'dictionary': 'DICT_4X4_50'
        }],
        remappings=[
            ('/camera_info', '/iiwa_camera/camera_info'),
            ('/image', '/iiwa_camera/image_raw'),
            ('/aruco_single/pose', '/aruco_single/pose')
        ],
        output='screen'
    )

    # --- NODO KDL (IL TUO CODICE C++) ---
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa',
        output='screen',
        parameters=[
            config_params,
            {
                'use_sim_time': True,
                'cmd_interface': LaunchConfiguration('cmd_interface'),
                'ctrl': LaunchConfiguration('ctrl'),
            }
        ],
        remappings=[
            ('joint_states', '/iiwa/joint_states'),
            ('velocity_controller/commands', '/iiwa/velocity_controller/commands'),
            # Mapping per il topic della posa del marker
            ('/aruco_single/pose', '/aruco_single/pose')
        ]
    )
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0.07',            # Coordinata X URDF
            '--y', '0.0',             # Coordinata Y URDF
            '--z', '-0.03',           # Coordinata Z URDF
            '--roll', '3.14',         # R (Roll) URDF
            '--pitch', '-1.57',       # P (Pitch) URDF
            '--yaw', '0.0',           # Y (Yaw) URDF
            '--frame-id', 'iiwa_tool0',                  # Padre (Parent Link nell'URDF)
            '--child-frame-id', 'iiwa/iiwa_link_7/iiwa_camera' # Figlio (Nome che dava errore)
        ],
        output='screen'
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        bridge_node,
        aruco_node,
        ros2_kdl_node,
        static_tf_node
    ])
