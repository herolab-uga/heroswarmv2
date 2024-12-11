// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:srv/GetCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__TRAITS_HPP_
#define ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/srv/detail/get_charger__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'name'
#include "std_msgs/msg/detail/string__traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCharger_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    to_flow_style_yaml(msg.name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCharger_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name:\n";
    to_block_style_yaml(msg.name, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCharger_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::srv::GetCharger_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::GetCharger_Request & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::GetCharger_Request>()
{
  return "robot_msgs::srv::GetCharger_Request";
}

template<>
inline const char * name<robot_msgs::srv::GetCharger_Request>()
{
  return "robot_msgs/srv/GetCharger_Request";
}

template<>
struct has_fixed_size<robot_msgs::srv::GetCharger_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::String>::value> {};

template<>
struct has_bounded_size<robot_msgs::srv::GetCharger_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::String>::value> {};

template<>
struct is_message<robot_msgs::srv::GetCharger_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'id'
#include "std_msgs/msg/detail/u_int16__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCharger_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    to_flow_style_yaml(msg.id, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCharger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id:\n";
    to_block_style_yaml(msg.id, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCharger_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace robot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_msgs::srv::GetCharger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::GetCharger_Response & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::GetCharger_Response>()
{
  return "robot_msgs::srv::GetCharger_Response";
}

template<>
inline const char * name<robot_msgs::srv::GetCharger_Response>()
{
  return "robot_msgs/srv/GetCharger_Response";
}

template<>
struct has_fixed_size<robot_msgs::srv::GetCharger_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<std_msgs::msg::UInt16>::value> {};

template<>
struct has_bounded_size<robot_msgs::srv::GetCharger_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<std_msgs::msg::UInt16>::value> {};

template<>
struct is_message<robot_msgs::srv::GetCharger_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_msgs::srv::GetCharger>()
{
  return "robot_msgs::srv::GetCharger";
}

template<>
inline const char * name<robot_msgs::srv::GetCharger>()
{
  return "robot_msgs/srv/GetCharger";
}

template<>
struct has_fixed_size<robot_msgs::srv::GetCharger>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_msgs::srv::GetCharger_Request>::value &&
    has_fixed_size<robot_msgs::srv::GetCharger_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_msgs::srv::GetCharger>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_msgs::srv::GetCharger_Request>::value &&
    has_bounded_size<robot_msgs::srv::GetCharger_Response>::value
  >
{
};

template<>
struct is_service<robot_msgs::srv::GetCharger>
  : std::true_type
{
};

template<>
struct is_service_request<robot_msgs::srv::GetCharger_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_msgs::srv::GetCharger_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__TRAITS_HPP_
