// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/battery_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: soc
  {
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
    out << ", ";
  }

  // member: charging
  {
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "soc: ";
    rosidl_generator_traits::value_to_yaml(msg.soc, out);
    out << "\n";
  }

  // member: charging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "charging: ";
    rosidl_generator_traits::value_to_yaml(msg.charging, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BatteryStatus & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::BatteryStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::BatteryStatus & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::BatteryStatus>()
{
  return "robot_msgs::msg::BatteryStatus";
}

template<>
inline const char * name<robot_msgs::msg::BatteryStatus>()
{
  return "robot_msgs/msg/BatteryStatus";
}

template<>
struct has_fixed_size<robot_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::BatteryStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::BatteryStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__TRAITS_HPP_
