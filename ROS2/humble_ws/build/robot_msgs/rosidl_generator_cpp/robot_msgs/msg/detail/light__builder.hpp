// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__LIGHT__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__LIGHT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/light__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_Light_gesture
{
public:
  explicit Init_Light_gesture(::robot_msgs::msg::Light & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::Light gesture(::robot_msgs::msg::Light::_gesture_type arg)
  {
    msg_.gesture = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::Light msg_;
};

class Init_Light_rgbw
{
public:
  Init_Light_rgbw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Light_gesture rgbw(::robot_msgs::msg::Light::_rgbw_type arg)
  {
    msg_.rgbw = std::move(arg);
    return Init_Light_gesture(msg_);
  }

private:
  ::robot_msgs::msg::Light msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::Light>()
{
  return robot_msgs::msg::builder::Init_Light_rgbw();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__LIGHT__BUILDER_HPP_
