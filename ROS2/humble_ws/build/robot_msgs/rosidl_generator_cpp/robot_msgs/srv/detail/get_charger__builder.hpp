// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:srv/GetCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__BUILDER_HPP_
#define ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/srv/detail/get_charger__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCharger_Request_name
{
public:
  Init_GetCharger_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::srv::GetCharger_Request name(::robot_msgs::srv::GetCharger_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::GetCharger_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::GetCharger_Request>()
{
  return robot_msgs::srv::builder::Init_GetCharger_Request_name();
}

}  // namespace robot_msgs


namespace robot_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCharger_Response_position
{
public:
  explicit Init_GetCharger_Response_position(::robot_msgs::srv::GetCharger_Response & msg)
  : msg_(msg)
  {}
  ::robot_msgs::srv::GetCharger_Response position(::robot_msgs::srv::GetCharger_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::srv::GetCharger_Response msg_;
};

class Init_GetCharger_Response_id
{
public:
  Init_GetCharger_Response_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetCharger_Response_position id(::robot_msgs::srv::GetCharger_Response::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GetCharger_Response_position(msg_);
  }

private:
  ::robot_msgs::srv::GetCharger_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::srv::GetCharger_Response>()
{
  return robot_msgs::srv::builder::Init_GetCharger_Response_id();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__BUILDER_HPP_
