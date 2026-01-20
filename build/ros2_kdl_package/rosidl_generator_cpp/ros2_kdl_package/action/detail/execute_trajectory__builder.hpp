// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_kdl_package:action/ExecuteTrajectory.idl
// generated code does not contain a copyright notice

#ifndef ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_
#define ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_kdl_package/action/detail/execute_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_Goal_joints_target
{
public:
  explicit Init_ExecuteTrajectory_Goal_joints_target(::ros2_kdl_package::action::ExecuteTrajectory_Goal & msg)
  : msg_(msg)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_Goal joints_target(::ros2_kdl_package::action::ExecuteTrajectory_Goal::_joints_target_type arg)
  {
    msg_.joints_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_Goal msg_;
};

class Init_ExecuteTrajectory_Goal_order
{
public:
  explicit Init_ExecuteTrajectory_Goal_order(::ros2_kdl_package::action::ExecuteTrajectory_Goal & msg)
  : msg_(msg)
  {}
  Init_ExecuteTrajectory_Goal_joints_target order(::ros2_kdl_package::action::ExecuteTrajectory_Goal::_order_type arg)
  {
    msg_.order = std::move(arg);
    return Init_ExecuteTrajectory_Goal_joints_target(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_Goal msg_;
};

class Init_ExecuteTrajectory_Goal_pose
{
public:
  Init_ExecuteTrajectory_Goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_Goal_order pose(::ros2_kdl_package::action::ExecuteTrajectory_Goal::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ExecuteTrajectory_Goal_order(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_Goal>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_Goal_pose();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_Result_success
{
public:
  Init_ExecuteTrajectory_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_Result success(::ros2_kdl_package::action::ExecuteTrajectory_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_Result>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_Result_success();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_Feedback_position_error
{
public:
  Init_ExecuteTrajectory_Feedback_position_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_Feedback position_error(::ros2_kdl_package::action::ExecuteTrajectory_Feedback::_position_error_type arg)
  {
    msg_.position_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_Feedback>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_Feedback_position_error();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_SendGoal_Request_goal
{
public:
  explicit Init_ExecuteTrajectory_SendGoal_Request_goal(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request goal(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request msg_;
};

class Init_ExecuteTrajectory_SendGoal_Request_goal_id
{
public:
  Init_ExecuteTrajectory_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_SendGoal_Request_goal goal_id(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteTrajectory_SendGoal_Request_goal(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Request>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_SendGoal_Request_goal_id();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_SendGoal_Response_stamp
{
public:
  explicit Init_ExecuteTrajectory_SendGoal_Response_stamp(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response stamp(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response msg_;
};

class Init_ExecuteTrajectory_SendGoal_Response_accepted
{
public:
  Init_ExecuteTrajectory_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_SendGoal_Response_stamp accepted(::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExecuteTrajectory_SendGoal_Response_stamp(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_SendGoal_Response>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_SendGoal_Response_accepted();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_GetResult_Request_goal_id
{
public:
  Init_ExecuteTrajectory_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Request goal_id(::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Request>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_GetResult_Request_goal_id();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_GetResult_Response_result
{
public:
  explicit Init_ExecuteTrajectory_GetResult_Response_result(::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response result(::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response msg_;
};

class Init_ExecuteTrajectory_GetResult_Response_status
{
public:
  Init_ExecuteTrajectory_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_GetResult_Response_result status(::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExecuteTrajectory_GetResult_Response_result(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_GetResult_Response>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_GetResult_Response_status();
}

}  // namespace ros2_kdl_package


namespace ros2_kdl_package
{

namespace action
{

namespace builder
{

class Init_ExecuteTrajectory_FeedbackMessage_feedback
{
public:
  explicit Init_ExecuteTrajectory_FeedbackMessage_feedback(::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage feedback(::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage msg_;
};

class Init_ExecuteTrajectory_FeedbackMessage_goal_id
{
public:
  Init_ExecuteTrajectory_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteTrajectory_FeedbackMessage_feedback goal_id(::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteTrajectory_FeedbackMessage_feedback(msg_);
  }

private:
  ::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_kdl_package::action::ExecuteTrajectory_FeedbackMessage>()
{
  return ros2_kdl_package::action::builder::Init_ExecuteTrajectory_FeedbackMessage_goal_id();
}

}  // namespace ros2_kdl_package

#endif  // ROS2_KDL_PACKAGE__ACTION__DETAIL__EXECUTE_TRAJECTORY__BUILDER_HPP_
