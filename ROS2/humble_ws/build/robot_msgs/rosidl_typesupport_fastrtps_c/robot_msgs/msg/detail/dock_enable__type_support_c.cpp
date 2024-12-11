// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/dock_enable__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_msgs/msg/detail/dock_enable__struct.h"
#include "robot_msgs/msg/detail/dock_enable__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _DockEnable__ros_msg_type = robot_msgs__msg__DockEnable;

static bool _DockEnable__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DockEnable__ros_msg_type * ros_message = static_cast<const _DockEnable__ros_msg_type *>(untyped_ros_message);
  // Field name: dock1
  {
    cdr << ros_message->dock1;
  }

  // Field name: dock2
  {
    cdr << ros_message->dock2;
  }

  // Field name: dock3
  {
    cdr << ros_message->dock3;
  }

  // Field name: dock4
  {
    cdr << ros_message->dock4;
  }

  return true;
}

static bool _DockEnable__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DockEnable__ros_msg_type * ros_message = static_cast<_DockEnable__ros_msg_type *>(untyped_ros_message);
  // Field name: dock1
  {
    cdr >> ros_message->dock1;
  }

  // Field name: dock2
  {
    cdr >> ros_message->dock2;
  }

  // Field name: dock3
  {
    cdr >> ros_message->dock3;
  }

  // Field name: dock4
  {
    cdr >> ros_message->dock4;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_robot_msgs__msg__DockEnable(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DockEnable__ros_msg_type * ros_message = static_cast<const _DockEnable__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name dock1
  {
    size_t item_size = sizeof(ros_message->dock1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dock2
  {
    size_t item_size = sizeof(ros_message->dock2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dock3
  {
    size_t item_size = sizeof(ros_message->dock3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dock4
  {
    size_t item_size = sizeof(ros_message->dock4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DockEnable__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_msgs__msg__DockEnable(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_robot_msgs__msg__DockEnable(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: dock1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: dock2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: dock3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: dock4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_msgs__msg__DockEnable;
    is_plain =
      (
      offsetof(DataType, dock4) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _DockEnable__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_msgs__msg__DockEnable(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_DockEnable = {
  "robot_msgs::msg",
  "DockEnable",
  _DockEnable__cdr_serialize,
  _DockEnable__cdr_deserialize,
  _DockEnable__get_serialized_size,
  _DockEnable__max_serialized_size
};

static rosidl_message_type_support_t _DockEnable__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DockEnable,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, msg, DockEnable)() {
  return &_DockEnable__type_support;
}

#if defined(__cplusplus)
}
#endif
