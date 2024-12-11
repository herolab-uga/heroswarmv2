// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__LIGHT__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__LIGHT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/light__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Light & msg,
  std::ostream & out)
{
  out << "{";
  // member: rgbw
  {
    if (msg.rgbw.size() == 0) {
      out << "rgbw: []";
    } else {
      out << "rgbw: [";
      size_t pending_items = msg.rgbw.size();
      for (auto item : msg.rgbw) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gesture
  {
    out << "gesture: ";
    rosidl_generator_traits::value_to_yaml(msg.gesture, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Light & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rgbw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rgbw.size() == 0) {
      out << "rgbw: []\n";
    } else {
      out << "rgbw:\n";
      for (auto item : msg.rgbw) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gesture
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gesture: ";
    rosidl_generator_traits::value_to_yaml(msg.gesture, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Light & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::Light & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::Light & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::Light>()
{
  return "robot_msgs::msg::Light";
}

template<>
inline const char * name<robot_msgs::msg::Light>()
{
  return "robot_msgs/msg/Light";
}

template<>
struct has_fixed_size<robot_msgs::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_msgs::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_msgs::msg::Light>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__LIGHT__TRAITS_HPP_
