# Final_project
In order to do the control vision:  
Run gazebo:
```
ros2 launch ros2_fra2mo warehouse.launch.py
```
Open rqt -> /iiwa_camera/image_raw.  
Run the kdl node: 
```
ros2 launch ros2_kdl_package launching.launch.py
```
Set the rqt topic to /aruco_singles/result.  
Run the client node 
```
ros2 run ros2_kdl_package ros2_kdl_node_client
```
