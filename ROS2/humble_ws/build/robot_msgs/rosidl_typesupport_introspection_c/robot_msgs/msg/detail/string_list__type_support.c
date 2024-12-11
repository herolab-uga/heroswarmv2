// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/msg/detail/string_list__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/msg/detail/string_list__functions.h"
#include "robot_msgs/msg/detail/string_list__struct.h"


// Include directives for member types
// Member `data`
#include "std_msgs/msg/string.h"
// Member `data`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__msg__StringList__init(message_memory);
}

void robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_fini_function(void * message_memory)
{
  robot_msgs__msg__StringList__fini(message_memory);
}

size_t robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__size_function__StringList__data(
  const void * untyped_member)
{
  const std_msgs__msg__String__Sequence * member =
    (const std_msgs__msg__String__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_const_function__StringList__data(
  const void * untyped_member, size_t index)
{
  const std_msgs__msg__String__Sequence * member =
    (const std_msgs__msg__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_function__StringList__data(
  void * untyped_member, size_t index)
{
  std_msgs__msg__String__Sequence * member =
    (std_msgs__msg__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__fetch_function__StringList__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const std_msgs__msg__String * item =
    ((const std_msgs__msg__String *)
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_const_function__StringList__data(untyped_member, index));
  std_msgs__msg__String * value =
    (std_msgs__msg__String *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__assign_function__StringList__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  std_msgs__msg__String * item =
    ((std_msgs__msg__String *)
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_function__StringList__data(untyped_member, index));
  const std_msgs__msg__String * value =
    (const std_msgs__msg__String *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__resize_function__StringList__data(
  void * untyped_member, size_t size)
{
  std_msgs__msg__String__Sequence * member =
    (std_msgs__msg__String__Sequence *)(untyped_member);
  std_msgs__msg__String__Sequence__fini(member);
  return std_msgs__msg__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__StringList, data),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__size_function__StringList__data,  // size() function pointer
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_const_function__StringList__data,  // get_const(index) function pointer
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__get_function__StringList__data,  // get(index) function pointer
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__fetch_function__StringList__data,  // fetch(index, &value) function pointer
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__assign_function__StringList__data,  // assign(index, value) function pointer
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__resize_function__StringList__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_members = {
  "robot_msgs__msg",  // message namespace
  "StringList",  // message name
  1,  // number of fields
  sizeof(robot_msgs__msg__StringList),
  robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_member_array,  // message members
  robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_type_support_handle = {
  0,
  &robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, msg, StringList)() {
  robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  if (!robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_type_support_handle.typesupport_identifier) {
    robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__msg__StringList__rosidl_typesupport_introspection_c__StringList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
