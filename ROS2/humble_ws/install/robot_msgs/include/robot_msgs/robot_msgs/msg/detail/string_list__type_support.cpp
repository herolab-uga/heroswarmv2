// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_msgs/msg/detail/string_list__struct.hpp"
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

void StringList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_msgs::msg::StringList(_init);
}

void StringList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_msgs::msg::StringList *>(message_memory);
  typed_message->~StringList();
}

size_t size_function__StringList__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std_msgs::msg::String> *>(untyped_member);
  return member->size();
}

const void * get_const_function__StringList__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std_msgs::msg::String> *>(untyped_member);
  return &member[index];
}

void * get_function__StringList__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std_msgs::msg::String> *>(untyped_member);
  return &member[index];
}

void fetch_function__StringList__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std_msgs::msg::String *>(
    get_const_function__StringList__data(untyped_member, index));
  auto & value = *reinterpret_cast<std_msgs::msg::String *>(untyped_value);
  value = item;
}

void assign_function__StringList__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std_msgs::msg::String *>(
    get_function__StringList__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const std_msgs::msg::String *>(untyped_value);
  item = value;
}

void resize_function__StringList__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std_msgs::msg::String> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember StringList_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::String>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs::msg::StringList, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__StringList__data,  // size() function pointer
    get_const_function__StringList__data,  // get_const(index) function pointer
    get_function__StringList__data,  // get(index) function pointer
    fetch_function__StringList__data,  // fetch(index, &value) function pointer
    assign_function__StringList__data,  // assign(index, value) function pointer
    resize_function__StringList__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers StringList_message_members = {
  "robot_msgs::msg",  // message namespace
  "StringList",  // message name
  1,  // number of fields
  sizeof(robot_msgs::msg::StringList),
  StringList_message_member_array,  // message members
  StringList_init_function,  // function to initialize message memory (memory has to be allocated)
  StringList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t StringList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &StringList_message_members,
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
get_message_type_support_handle<robot_msgs::msg::StringList>()
{
  return &::robot_msgs::msg::rosidl_typesupport_introspection_cpp::StringList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_msgs, msg, StringList)() {
  return &::robot_msgs::msg::rosidl_typesupport_introspection_cpp::StringList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
