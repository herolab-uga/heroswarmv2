// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/sensor_enable__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_SensorEnable_mic
{
public:
  explicit Init_SensorEnable_mic(::robot_msgs::msg::SensorEnable & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::SensorEnable mic(::robot_msgs::msg::SensorEnable::_mic_type arg)
  {
    msg_.mic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::SensorEnable msg_;
};

class Init_SensorEnable_proximity
{
public:
  explicit Init_SensorEnable_proximity(::robot_msgs::msg::SensorEnable & msg)
  : msg_(msg)
  {}
  Init_SensorEnable_mic proximity(::robot_msgs::msg::SensorEnable::_proximity_type arg)
  {
    msg_.proximity = std::move(arg);
    return Init_SensorEnable_mic(msg_);
  }

private:
  ::robot_msgs::msg::SensorEnable msg_;
};

class Init_SensorEnable_light
{
public:
  explicit Init_SensorEnable_light(::robot_msgs::msg::SensorEnable & msg)
  : msg_(msg)
  {}
  Init_SensorEnable_proximity light(::robot_msgs::msg::SensorEnable::_light_type arg)
  {
    msg_.light = std::move(arg);
    return Init_SensorEnable_proximity(msg_);
  }

private:
  ::robot_msgs::msg::SensorEnable msg_;
};

class Init_SensorEnable_imu
{
public:
  explicit Init_SensorEnable_imu(::robot_msgs::msg::SensorEnable & msg)
  : msg_(msg)
  {}
  Init_SensorEnable_light imu(::robot_msgs::msg::SensorEnable::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_SensorEnable_light(msg_);
  }

private:
  ::robot_msgs::msg::SensorEnable msg_;
};

class Init_SensorEnable_environment
{
public:
  Init_SensorEnable_environment()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorEnable_imu environment(::robot_msgs::msg::SensorEnable::_environment_type arg)
  {
    msg_.environment = std::move(arg);
    return Init_SensorEnable_imu(msg_);
  }

private:
  ::robot_msgs::msg::SensorEnable msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::SensorEnable>()
{
  return robot_msgs::msg::builder::Init_SensorEnable_environment();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__BUILDER_HPP_
