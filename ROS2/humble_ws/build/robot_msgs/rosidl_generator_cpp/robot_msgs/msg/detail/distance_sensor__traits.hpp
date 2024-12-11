// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:msg/DistanceSensor.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__TRAITS_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/msg/detail/distance_sensor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DistanceSensor & msg,
  std::ostream & out)
{
  out << "{";
  // member: sensor1
  {
    out << "sensor1: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor1, out);
    out << ", ";
  }

  // member: sensor2
  {
    out << "sensor2: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor2, out);
    out << ", ";
  }

  // member: sensor3
  {
    out << "sensor3: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor3, out);
    out << ", ";
  }

  // member: sensor4
  {
    out << "sensor4: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DistanceSensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sensor1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor1: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor1, out);
    out << "\n";
  }

  // member: sensor2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor2: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor2, out);
    out << "\n";
  }

  // member: sensor3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor3: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor3, out);
    out << "\n";
  }

  // member: sensor4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensor4: ";
    rosidl_generator_traits::value_to_yaml(msg.sensor4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DistanceSensor & msg, bool use_flow_style = false)
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
  const robot_msgs::msg::DistanceSensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::msg::DistanceSensor & msg)
{
  return robot_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::msg::DistanceSensor>()
{
  return "robot_msgs::msg::DistanceSensor";
}

template<>
inline const char * name<robot_msgs::msg::DistanceSensor>()
{
  return "robot_msgs/msg/DistanceSensor";
}

template<>
struct has_fixed_size<robot_msgs::msg::DistanceSensor>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_msgs::msg::DistanceSensor>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_msgs::msg::DistanceSensor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__TRAITS_HPP_
