// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_BatteryStatus_charging
{
public:
  explicit Init_BatteryStatus_charging(::robot_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::BatteryStatus charging(::robot_msgs::msg::BatteryStatus::_charging_type arg)
  {
    msg_.charging = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_soc
{
public:
  explicit Init_BatteryStatus_soc(::robot_msgs::msg::BatteryStatus & msg)
  : msg_(msg)
  {}
  Init_BatteryStatus_charging soc(::robot_msgs::msg::BatteryStatus::_soc_type arg)
  {
    msg_.soc = std::move(arg);
    return Init_BatteryStatus_charging(msg_);
  }

private:
  ::robot_msgs::msg::BatteryStatus msg_;
};

class Init_BatteryStatus_voltage
{
public:
  Init_BatteryStatus_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BatteryStatus_soc voltage(::robot_msgs::msg::BatteryStatus::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_BatteryStatus_soc(msg_);
  }

private:
  ::robot_msgs::msg::BatteryStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::BatteryStatus>()
{
  return robot_msgs::msg::builder::Init_BatteryStatus_voltage();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__BUILDER_HPP_
