// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:msg/Light.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/msg/detail/light__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/msg/detail/light__functions.h"
#include "robot_msgs/msg/detail/light__struct.h"


// Include directives for member types
// Member `rgbw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__msg__Light__init(message_memory);
}

void robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_fini_function(void * message_memory)
{
  robot_msgs__msg__Light__fini(message_memory);
}

size_t robot_msgs__msg__Light__rosidl_typesupport_introspection_c__size_function__Light__rgbw(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_const_function__Light__rgbw(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_function__Light__rgbw(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__Light__rosidl_typesupport_introspection_c__fetch_function__Light__rgbw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_const_function__Light__rgbw(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__Light__rosidl_typesupport_introspection_c__assign_function__Light__rgbw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_function__Light__rgbw(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__Light__rosidl_typesupport_introspection_c__resize_function__Light__rgbw(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_member_array[2] = {
  {
    "rgbw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__Light, rgbw),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__size_function__Light__rgbw,  // size() function pointer
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_const_function__Light__rgbw,  // get_const(index) function pointer
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__get_function__Light__rgbw,  // get(index) function pointer
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__fetch_function__Light__rgbw,  // fetch(index, &value) function pointer
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__assign_function__Light__rgbw,  // assign(index, value) function pointer
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__resize_function__Light__rgbw  // resize(index) function pointer
  },
  {
    "gesture",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__Light, gesture),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_members = {
  "robot_msgs__msg",  // message namespace
  "Light",  // message name
  2,  // number of fields
  sizeof(robot_msgs__msg__Light),
  robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_member_array,  // message members
  robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_type_support_handle = {
  0,
  &robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, msg, Light)() {
  if (!robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_type_support_handle.typesupport_identifier) {
    robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__msg__Light__rosidl_typesupport_introspection_c__Light_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
