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
ros2 topic pub /iiwa/velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0]}"

```
