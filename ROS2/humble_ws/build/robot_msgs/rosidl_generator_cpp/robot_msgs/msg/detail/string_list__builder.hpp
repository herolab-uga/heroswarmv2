// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__STRING_LIST__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__STRING_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/string_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_StringList_data
{
public:
  Init_StringList_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_msgs::msg::StringList data(::robot_msgs::msg::StringList::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::StringList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::StringList>()
{
  return robot_msgs::msg::builder::Init_StringList_data();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__STRING_LIST__BUILDER_HPP_
