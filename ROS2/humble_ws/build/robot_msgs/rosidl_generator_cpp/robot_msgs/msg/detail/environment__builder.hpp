// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/environment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_Environment_altitude
{
public:
  explicit Init_Environment_altitude(::robot_msgs::msg::Environment & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::Environment altitude(::robot_msgs::msg::Environment::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::Environment msg_;
};

class Init_Environment_humidity
{
public:
  explicit Init_Environment_humidity(::robot_msgs::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_altitude humidity(::robot_msgs::msg::Environment::_humidity_type arg)
  {
    msg_.humidity = std::move(arg);
    return Init_Environment_altitude(msg_);
  }

private:
  ::robot_msgs::msg::Environment msg_;
};

class Init_Environment_pressure
{
public:
  explicit Init_Environment_pressure(::robot_msgs::msg::Environment & msg)
  : msg_(msg)
  {}
  Init_Environment_humidity pressure(::robot_msgs::msg::Environment::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_Environment_humidity(msg_);
  }

private:
  ::robot_msgs::msg::Environment msg_;
};

class Init_Environment_temp
{
public:
  Init_Environment_temp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Environment_pressure temp(::robot_msgs::msg::Environment::_temp_type arg)
  {
    msg_.temp = std::move(arg);
    return Init_Environment_pressure(msg_);
  }

private:
  ::robot_msgs::msg::Environment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::Environment>()
{
  return robot_msgs::msg::builder::Init_Environment_temp();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__BUILDER_HPP_
