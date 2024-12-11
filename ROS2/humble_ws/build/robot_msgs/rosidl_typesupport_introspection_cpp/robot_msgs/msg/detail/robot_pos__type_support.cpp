// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_msgs/msg/detail/robot_pos__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robot_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RobotPos_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_msgs::msg::RobotPos(_init);
}

void RobotPos_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_msgs::msg::RobotPos *>(message_memory);
  typed_message->~RobotPos();
}

size_t size_function__RobotPos__robot_pos(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPos__robot_pos(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPos__robot_pos(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPos__robot_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const nav_msgs::msg::Odometry *>(
    get_const_function__RobotPos__robot_pos(untyped_member, index));
  auto & value = *reinterpret_cast<nav_msgs::msg::Odometry *>(untyped_value);
  value = item;
}

void assign_function__RobotPos__robot_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<nav_msgs::msg::Odometry *>(
    get_function__RobotPos__robot_pos(untyped_member, index));
  const auto & value = *reinterpret_cast<const nav_msgs::msg::Odometry *>(untyped_value);
  item = value;
}

void resize_function__RobotPos__robot_pos(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<nav_msgs::msg::Odometry> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotPos_message_member_array[1] = {
  {
    "robot_pos",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<nav_msgs::msg::Odometry>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs::msg::RobotPos, robot_pos),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPos__robot_pos,  // size() function pointer
    get_const_function__RobotPos__robot_pos,  // get_const(index) function pointer
    get_function__RobotPos__robot_pos,  // get(index) function pointer
    fetch_function__RobotPos__robot_pos,  // fetch(index, &value) function pointer
    assign_function__RobotPos__robot_pos,  // assign(index, value) function pointer
    resize_function__RobotPos__robot_pos  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotPos_message_members = {
  "robot_msgs::msg",  // message namespace
  "RobotPos",  // message name
  1,  // number of fields
  sizeof(robot_msgs::msg::RobotPos),
  RobotPos_message_member_array,  // message members
  RobotPos_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotPos_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotPos_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotPos_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robot_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_msgs::msg::RobotPos>()
{
  return &::robot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPos_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, msg, RobotPos)() {
  return &::robot_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPos_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
