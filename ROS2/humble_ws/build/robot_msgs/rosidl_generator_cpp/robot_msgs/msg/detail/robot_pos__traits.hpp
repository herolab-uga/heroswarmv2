// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/robot_pos__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'robot_pos'
#include "nav_msgs/msg/detail/odometry__traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPos & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_pos
  {
    if (msg.robot_pos.size() == 0) {
      out << "robot_pos: []";
    } else {
      out << "robot_pos: [";
      size_t pending_items = msg.robot_pos.size();
      for (auto item : msg.robot_pos) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotPos & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_pos.size() == 0) {
      out << "robot_pos: []\n";
    } else {
      out << "robot_pos:\n";
      for (auto item : msg.robot_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotPos & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::msg::RobotPos & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::RobotPos & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::RobotPos>()
{
  return "robot_msgs::msg::RobotPos";
}

template<>
inline const char * name<robot_msgs::msg::RobotPos>()
{
  return "robot_msgs/msg/RobotPos";
}

template<>
struct has_fixed_size<robot_msgs::msg::RobotPos>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::RobotPos>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::RobotPos>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__TRAITS_HPP_
