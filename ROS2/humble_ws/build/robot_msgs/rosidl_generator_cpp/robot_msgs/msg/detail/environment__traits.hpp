// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/environment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Environment & msg,
  std::ostream & out)
{
  out << "{";
  // member: temp
  {
    out << "temp: ";
    rosidl_generator_traits::value_to_yaml(msg.temp, out);
    out << ", ";
  }

  // member: pressure
  {
    out << "pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure, out);
    out << ", ";
  }

  // member: humidity
  {
    out << "humidity: ";
    rosidl_generator_traits::value_to_yaml(msg.humidity, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Environment & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temp: ";
    rosidl_generator_traits::value_to_yaml(msg.temp, out);
    out << "\n";
  }

  // member: pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure, out);
    out << "\n";
  }

  // member: humidity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "humidity: ";
    rosidl_generator_traits::value_to_yaml(msg.humidity, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Environment & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::Environment & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::Environment & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::Environment>()
{
  return "robot_msgs::msg::Environment";
}

template<>
inline const char * name<robot_msgs::msg::Environment>()
{
  return "robot_msgs/msg/Environment";
}

template<>
struct has_fixed_size<robot_msgs::msg::Environment>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::Environment>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::Environment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__TRAITS_HPP_
