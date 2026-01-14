cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

# ======== Opzioni compilatore ========
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======== Find dependencies ========
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)             # Aggiunto tf2 base
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ======== Generazione action ========
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs # geometry_msgs aggiunto qui è fondamentale
)
ament_export_dependencies(rosidl_default_runtime)

# ======== Nodo principale (SERVER) ========
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)

target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node
  orocos_kdl
  rclcpp
  rclcpp_action
  action_msgs
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ======== Nodo CLIENT (Corretto) ========
add_executable(ros2_kdl_node_client src/ros2_kdl_node_client.cpp)

ament_target_dependencies(ros2_kdl_node_client
  rclcpp
  rclcpp_action
  geometry_msgs       # <--- Aggiunto
  std_msgs            # <--- Aggiunto
  tf2                 # <--- Aggiunto
  tf2_ros             # <--- Aggiunto
  tf2_geometry_msgs   # <--- Aggiunto
)

# ======== Nodo Esempio (Opzionale) ========
add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_example orocos_kdl rclcpp)

# ======== Collegamento Action (Metodo Moderno per evitare Warning) ========
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(ros2_kdl_node "${cpp_typesupport_target}")
target_link_libraries(ros2_kdl_node_client "${cpp_typesupport_target}")


# ======== Install ========
install(TARGETS ros2_kdl_node ros2_kdl_node_example ros2_kdl_node_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ======== Testing ========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
