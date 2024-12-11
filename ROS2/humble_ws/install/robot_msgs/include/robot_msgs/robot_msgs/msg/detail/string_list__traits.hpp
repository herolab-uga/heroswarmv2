// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__STRING_LIST__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__STRING_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/string_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'data'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const StringList & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const StringList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringList & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::StringList & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::StringList & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::StringList>()
{
  return "robot_msgs::msg::StringList";
}

template<>
inline const char * name<robot_msgs::msg::StringList>()
{
  return "robot_msgs/msg/StringList";
}

template<>
struct has_fixed_size<robot_msgs::msg::StringList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::StringList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::StringList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__STRING_LIST__TRAITS_HPP_
