// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/sensor_enable__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_msgs/msg/detail/sensor_enable__struct.h"
#include "robot_msgs/msg/detail/sensor_enable__functions.h"
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


using _SensorEnable__ros_msg_type = robot_msgs__msg__SensorEnable;

static bool _SensorEnable__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SensorEnable__ros_msg_type * ros_message = static_cast<const _SensorEnable__ros_msg_type *>(untyped_ros_message);
  // Field name: environment
  {
    cdr << (ros_message->environment ? true : false);
  }

  // Field name: imu
  {
    cdr << (ros_message->imu ? true : false);
  }

  // Field name: light
  {
    cdr << (ros_message->light ? true : false);
  }

  // Field name: proximity
  {
    cdr << (ros_message->proximity ? true : false);
  }

  // Field name: mic
  {
    cdr << (ros_message->mic ? true : false);
  }

  return true;
}

static bool _SensorEnable__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SensorEnable__ros_msg_type * ros_message = static_cast<_SensorEnable__ros_msg_type *>(untyped_ros_message);
  // Field name: environment
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->environment = tmp ? true : false;
  }

  // Field name: imu
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->imu = tmp ? true : false;
  }

  // Field name: light
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->light = tmp ? true : false;
  }

  // Field name: proximity
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->proximity = tmp ? true : false;
  }

  // Field name: mic
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->mic = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_robot_msgs__msg__SensorEnable(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SensorEnable__ros_msg_type * ros_message = static_cast<const _SensorEnable__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name environment
  {
    size_t item_size = sizeof(ros_message->environment);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name imu
  {
    size_t item_size = sizeof(ros_message->imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light
  {
    size_t item_size = sizeof(ros_message->light);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name proximity
  {
    size_t item_size = sizeof(ros_message->proximity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mic
  {
    size_t item_size = sizeof(ros_message->mic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SensorEnable__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_msgs__msg__SensorEnable(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_robot_msgs__msg__SensorEnable(
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

  // member: environment
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: light
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: proximity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: mic
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
    using DataType = robot_msgs__msg__SensorEnable;
    is_plain =
      (
      offsetof(DataType, mic) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SensorEnable__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_msgs__msg__SensorEnable(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SensorEnable = {
  "robot_msgs::msg",
  "SensorEnable",
  _SensorEnable__cdr_serialize,
  _SensorEnable__cdr_deserialize,
  _SensorEnable__get_serialized_size,
  _SensorEnable__max_serialized_size
};

static rosidl_message_type_support_t _SensorEnable__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SensorEnable,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, msg, SensorEnable)() {
  return &_SensorEnable__type_support;
}

#if defined(__cplusplus)
}
#endif
