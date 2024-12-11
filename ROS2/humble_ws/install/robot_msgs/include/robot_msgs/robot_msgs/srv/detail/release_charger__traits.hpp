// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_msgs:srv/ReleaseCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__TRAITS_HPP_
#define ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_msgs/srv/detail/release_charger__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'id'
#include "std_msgs/msg/detail/u_int16__traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ReleaseCharger_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    to_flow_style_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReleaseCharger_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReleaseCharger_Request & msg, bool use_flow_style = false)
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
  const robot_msgs::srv::ReleaseCharger_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::ReleaseCharger_Request & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::ReleaseCharger_Request>()
{
  return "robot_msgs::srv::ReleaseCharger_Request";
}

template<>
inline const char * name<robot_msgs::srv::ReleaseCharger_Request>()
{
  return "robot_msgs/srv/ReleaseCharger_Request";
}

template<>
struct has_fixed_size<robot_msgs::srv::ReleaseCharger_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::UInt16>::value> {};

template<>
struct has_bounded_size<robot_msgs::srv::ReleaseCharger_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::UInt16>::value> {};

template<>
struct is_message<robot_msgs::srv::ReleaseCharger_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'released'
#include "std_msgs/msg/detail/bool__traits.hpp"

namespace robot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ReleaseCharger_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: released
  {
    out << "released: ";
    to_flow_style_yaml(msg.released, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReleaseCharger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: released
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "released:\n";
    to_block_style_yaml(msg.released, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReleaseCharger_Response & msg, bool use_flow_style = false)
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
  const robot_msgs::srv::ReleaseCharger_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const robot_msgs::srv::ReleaseCharger_Response & msg)
{
  return robot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<robot_msgs::srv::ReleaseCharger_Response>()
{
  return "robot_msgs::srv::ReleaseCharger_Response";
}

template<>
inline const char * name<robot_msgs::srv::ReleaseCharger_Response>()
{
  return "robot_msgs/srv/ReleaseCharger_Response";
}

template<>
struct has_fixed_size<robot_msgs::srv::ReleaseCharger_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Bool>::value> {};

template<>
struct has_bounded_size<robot_msgs::srv::ReleaseCharger_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Bool>::value> {};

template<>
struct is_message<robot_msgs::srv::ReleaseCharger_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_msgs::srv::ReleaseCharger>()
{
  return "robot_msgs::srv::ReleaseCharger";
}

template<>
inline const char * name<robot_msgs::srv::ReleaseCharger>()
{
  return "robot_msgs/srv/ReleaseCharger";
}

template<>
struct has_fixed_size<robot_msgs::srv::ReleaseCharger>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_msgs::srv::ReleaseCharger_Request>::value &&
    has_fixed_size<robot_msgs::srv::ReleaseCharger_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_msgs::srv::ReleaseCharger>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_msgs::srv::ReleaseCharger_Request>::value &&
    has_bounded_size<robot_msgs::srv::ReleaseCharger_Response>::value
  >
{
};

template<>
struct is_service<robot_msgs::srv::ReleaseCharger>
  : std::true_type
{
};

template<>
struct is_service_request<robot_msgs::srv::ReleaseCharger_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_msgs::srv::ReleaseCharger_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__TRAITS_HPP_
