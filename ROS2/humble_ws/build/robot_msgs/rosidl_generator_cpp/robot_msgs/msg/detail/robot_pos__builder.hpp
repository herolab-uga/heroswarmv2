// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/robot_pos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotPos_robot_pos
{
public:
  Init_RobotPos_robot_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::msg::RobotPos robot_pos(::robot_msgs::msg::RobotPos::_robot_pos_type arg)
  {
    msg_.robot_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::RobotPos msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::RobotPos>()
{
  return robot_msgs::msg::builder::Init_RobotPos_robot_pos();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__BUILDER_HPP_
