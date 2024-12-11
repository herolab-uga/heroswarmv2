// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/DistanceSensor.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/distance_sensor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_DistanceSensor_sensor4
{
public:
  explicit Init_DistanceSensor_sensor4(::robot_msgs::msg::DistanceSensor & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::DistanceSensor sensor4(::robot_msgs::msg::DistanceSensor::_sensor4_type arg)
  {
    msg_.sensor4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::DistanceSensor msg_;
};

class Init_DistanceSensor_sensor3
{
public:
  explicit Init_DistanceSensor_sensor3(::robot_msgs::msg::DistanceSensor & msg)
  : msg_(msg)
  {}
  Init_DistanceSensor_sensor4 sensor3(::robot_msgs::msg::DistanceSensor::_sensor3_type arg)
  {
    msg_.sensor3 = std::move(arg);
    return Init_DistanceSensor_sensor4(msg_);
  }

private:
  ::robot_msgs::msg::DistanceSensor msg_;
};

class Init_DistanceSensor_sensor2
{
public:
  explicit Init_DistanceSensor_sensor2(::robot_msgs::msg::DistanceSensor & msg)
  : msg_(msg)
  {}
  Init_DistanceSensor_sensor3 sensor2(::robot_msgs::msg::DistanceSensor::_sensor2_type arg)
  {
    msg_.sensor2 = std::move(arg);
    return Init_DistanceSensor_sensor3(msg_);
  }

private:
  ::robot_msgs::msg::DistanceSensor msg_;
};

class Init_DistanceSensor_sensor1
{
public:
  Init_DistanceSensor_sensor1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DistanceSensor_sensor2 sensor1(::robot_msgs::msg::DistanceSensor::_sensor1_type arg)
  {
    msg_.sensor1 = std::move(arg);
    return Init_DistanceSensor_sensor2(msg_);
  }

private:
  ::robot_msgs::msg::DistanceSensor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::DistanceSensor>()
{
  return robot_msgs::msg::builder::Init_DistanceSensor_sensor1();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__BUILDER_HPP_
