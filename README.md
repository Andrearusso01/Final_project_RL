# Final_project
Run gazebo:
```
ros2 launch ros2_fra2mo warehouse.launch.py
```
To set a goal with autonomous navigation:
```
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
In order to do the control vision:  
Open rqt -> /iiwa_camera/image_raw.  
Run the kdl node: 
```
ros2 launch ros2_kdl_package launching.launch.py ctrl:=vision
```
Set the rqt topic to /aruco_single/result.  
Run the client node 
```
ros2 run ros2_kdl_package ros2_kdl_node_client
```
To use the detachable joint:
```
ign topic -t /gripper/detach -m ignition.msgs.Empty -p ''
```
```
ign topic -t /gripper/attach -m ignition.msgs.Empty -p ''
```
To move iiwa:
```
ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [-1.57, 0.5, 0.0, -1.2, 0.0, 1.57, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    },
    {
      positions: [-1.57, 1.05, 0.0, -0.85, 0.0, 1.57, 0.0],
      time_from_start: {sec: 8, nanosec: 0}
    }
  ]
}"


```
