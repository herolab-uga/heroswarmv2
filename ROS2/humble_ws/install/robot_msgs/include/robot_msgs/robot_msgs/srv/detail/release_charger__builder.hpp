// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/ReleaseCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/release_charger__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_ReleaseCharger_Request_id
{
public:
  Init_ReleaseCharger_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::srv::ReleaseCharger_Request id(::robot_msgs::srv::ReleaseCharger_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::ReleaseCharger_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::ReleaseCharger_Request>()
{
  return robot_msgs::srv::builder::Init_ReleaseCharger_Request_id();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_ReleaseCharger_Response_released
{
public:
  Init_ReleaseCharger_Response_released()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::srv::ReleaseCharger_Response released(::robot_msgs::srv::ReleaseCharger_Response::_released_type arg)
  {
    msg_.released = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::ReleaseCharger_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::ReleaseCharger_Response>()
{
  return robot_msgs::srv::builder::Init_ReleaseCharger_Response_released();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__BUILDER_HPP_
