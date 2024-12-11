// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/dock_enable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_DockEnable_dock4
{
public:
  explicit Init_DockEnable_dock4(::robot_msgs::msg::DockEnable & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::DockEnable dock4(::robot_msgs::msg::DockEnable::_dock4_type arg)
  {
    msg_.dock4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::DockEnable msg_;
};

class Init_DockEnable_dock3
{
public:
  explicit Init_DockEnable_dock3(::robot_msgs::msg::DockEnable & msg)
  : msg_(msg)
  {}
  Init_DockEnable_dock4 dock3(::robot_msgs::msg::DockEnable::_dock3_type arg)
  {
    msg_.dock3 = std::move(arg);
    return Init_DockEnable_dock4(msg_);
  }

private:
  ::robot_msgs::msg::DockEnable msg_;
};

class Init_DockEnable_dock2
{
public:
  explicit Init_DockEnable_dock2(::robot_msgs::msg::DockEnable & msg)
  : msg_(msg)
  {}
  Init_DockEnable_dock3 dock2(::robot_msgs::msg::DockEnable::_dock2_type arg)
  {
    msg_.dock2 = std::move(arg);
    return Init_DockEnable_dock3(msg_);
  }

private:
  ::robot_msgs::msg::DockEnable msg_;
};

class Init_DockEnable_dock1
{
public:
  Init_DockEnable_dock1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DockEnable_dock2 dock1(::robot_msgs::msg::DockEnable::_dock1_type arg)
  {
    msg_.dock1 = std::move(arg);
    return Init_DockEnable_dock2(msg_);
  }

private:
  ::robot_msgs::msg::DockEnable msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::DockEnable>()
{
  return robot_msgs::msg::builder::Init_DockEnable_dock1();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__BUILDER_HPP_
