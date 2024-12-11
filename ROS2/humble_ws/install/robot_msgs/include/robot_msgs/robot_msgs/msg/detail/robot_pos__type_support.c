// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/msg/detail/robot_pos__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/msg/detail/robot_pos__functions.h"
#include "robot_msgs/msg/detail/robot_pos__struct.h"


// Include directives for member types
// Member `robot_pos`
#include "nav_msgs/msg/odometry.h"
// Member `robot_pos`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__msg__RobotPos__init(message_memory);
}

void robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_fini_function(void * message_memory)
{
  robot_msgs__msg__RobotPos__fini(message_memory);
}

size_t robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__size_function__RobotPos__robot_pos(
  const void * untyped_member)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_const_function__RobotPos__robot_pos(
  const void * untyped_member, size_t index)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_function__RobotPos__robot_pos(
  void * untyped_member, size_t index)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__fetch_function__RobotPos__robot_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nav_msgs__msg__Odometry * item =
    ((const nav_msgs__msg__Odometry *)
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_const_function__RobotPos__robot_pos(untyped_member, index));
  nav_msgs__msg__Odometry * value =
    (nav_msgs__msg__Odometry *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__assign_function__RobotPos__robot_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nav_msgs__msg__Odometry * item =
    ((nav_msgs__msg__Odometry *)
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_function__RobotPos__robot_pos(untyped_member, index));
  const nav_msgs__msg__Odometry * value =
    (const nav_msgs__msg__Odometry *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__resize_function__RobotPos__robot_pos(
  void * untyped_member, size_t size)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  nav_msgs__msg__Odometry__Sequence__fini(member);
  return nav_msgs__msg__Odometry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_member_array[1] = {
  {
    "robot_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__RobotPos, robot_pos),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__size_function__RobotPos__robot_pos,  // size() function pointer
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_const_function__RobotPos__robot_pos,  // get_const(index) function pointer
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__get_function__RobotPos__robot_pos,  // get(index) function pointer
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__fetch_function__RobotPos__robot_pos,  // fetch(index, &value) function pointer
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__assign_function__RobotPos__robot_pos,  // assign(index, value) function pointer
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__resize_function__RobotPos__robot_pos  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_members = {
  "robot_msgs__msg",  // message namespace
  "RobotPos",  // message name
  1,  // number of fields
  sizeof(robot_msgs__msg__RobotPos),
  robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_member_array,  // message members
  robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_type_support_handle = {
  0,
  &robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, msg, RobotPos)() {
  robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  if (!robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_type_support_handle.typesupport_identifier) {
    robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__msg__RobotPos__rosidl_typesupport_introspection_c__RobotPos_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
