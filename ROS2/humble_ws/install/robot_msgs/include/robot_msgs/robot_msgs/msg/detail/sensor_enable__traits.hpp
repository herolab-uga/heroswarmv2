// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/sensor_enable__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorEnable & msg,
  std::ostream & out)
{
  out << "{";
  // member: environment
  {
    out << "environment: ";
    rosidl_generator_traits::value_to_yaml(msg.environment, out);
    out << ", ";
  }

  // member: imu
  {
    out << "imu: ";
    rosidl_generator_traits::value_to_yaml(msg.imu, out);
    out << ", ";
  }

  // member: light
  {
    out << "light: ";
    rosidl_generator_traits::value_to_yaml(msg.light, out);
    out << ", ";
  }

  // member: proximity
  {
    out << "proximity: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity, out);
    out << ", ";
  }

  // member: mic
  {
    out << "mic: ";
    rosidl_generator_traits::value_to_yaml(msg.mic, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: environment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "environment: ";
    rosidl_generator_traits::value_to_yaml(msg.environment, out);
    out << "\n";
  }

  // member: imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu: ";
    rosidl_generator_traits::value_to_yaml(msg.imu, out);
    out << "\n";
  }

  // member: light
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "light: ";
    rosidl_generator_traits::value_to_yaml(msg.light, out);
    out << "\n";
  }

  // member: proximity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "proximity: ";
    rosidl_generator_traits::value_to_yaml(msg.proximity, out);
    out << "\n";
  }

  // member: mic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mic: ";
    rosidl_generator_traits::value_to_yaml(msg.mic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorEnable & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::SensorEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::SensorEnable & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::SensorEnable>()
{
  return "robot_msgs::msg::SensorEnable";
}

template<>
inline const char * name<robot_msgs::msg::SensorEnable>()
{
  return "robot_msgs/msg/SensorEnable";
}

template<>
struct has_fixed_size<robot_msgs::msg::SensorEnable>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::SensorEnable>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::SensorEnable>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__TRAITS_HPP_
