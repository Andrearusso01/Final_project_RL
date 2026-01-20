// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_kdl_package:action/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

#ifndef ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_
#define ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'joints_target'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Goal
{
  geometry_msgs__msg__Pose pose;
  int32_t order;
  /// <--- AGGIUNTO: per i giunti del tavolo
  rosidl_runtime_c__double__Sequence joints_target;
} ros2_kdl_package__action__ExecuteTrajectory_Goal;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_Goal.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Goal__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Result
{
  bool success;
} ros2_kdl_package__action__ExecuteTrajectory_Result;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_Result.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Result__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Feedback
{
  double position_error[3];
} ros2_kdl_package__action__ExecuteTrajectory_Feedback;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_Feedback.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_Feedback__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ros2_kdl_package/action/detail/execute_trajectory__struct.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ros2_kdl_package__action__ExecuteTrajectory_Goal goal;
} ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ros2_kdl_package/action/detail/execute_trajectory__struct.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response
{
  int8_t status;
  ros2_kdl_package__action__ExecuteTrajectory_Result result;
} ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ros2_kdl_package/action/detail/execute_trajectory__struct.h"

/// Struct defined in action/ExecuteTrajectory in the package ros2_kdl_package.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ros2_kdl_package__action__ExecuteTrajectory_Feedback feedback;
} ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage;

// Struct for a sequence of ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage.
typedef struct ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage__Sequence
{
  ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_kdl_package__action__ExecuteTrajectory_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__STRUCT_H_
