// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/dock_enable__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DockEnable & msg,
  std::ostream & out)
{
  out << "{";
  // member: dock1
  {
    out << "dock1: ";
    rosidl_generator_traits::value_to_yaml(msg.dock1, out);
    out << ", ";
  }

  // member: dock2
  {
    out << "dock2: ";
    rosidl_generator_traits::value_to_yaml(msg.dock2, out);
    out << ", ";
  }

  // member: dock3
  {
    out << "dock3: ";
    rosidl_generator_traits::value_to_yaml(msg.dock3, out);
    out << ", ";
  }

  // member: dock4
  {
    out << "dock4: ";
    rosidl_generator_traits::value_to_yaml(msg.dock4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DockEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dock1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dock1: ";
    rosidl_generator_traits::value_to_yaml(msg.dock1, out);
    out << "\n";
  }

  // member: dock2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dock2: ";
    rosidl_generator_traits::value_to_yaml(msg.dock2, out);
    out << "\n";
  }

  // member: dock3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dock3: ";
    rosidl_generator_traits::value_to_yaml(msg.dock3, out);
    out << "\n";
  }

  // member: dock4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dock4: ";
    rosidl_generator_traits::value_to_yaml(msg.dock4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DockEnable & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::DockEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::DockEnable & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::DockEnable>()
{
  return "robot_msgs::msg::DockEnable";
}

template<>
inline const char * name<robot_msgs::msg::DockEnable>()
{
  return "robot_msgs/msg/DockEnable";
}

template<>
struct has_fixed_size<robot_msgs::msg::DockEnable>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::DockEnable>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::DockEnable>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__TRAITS_HPP_
