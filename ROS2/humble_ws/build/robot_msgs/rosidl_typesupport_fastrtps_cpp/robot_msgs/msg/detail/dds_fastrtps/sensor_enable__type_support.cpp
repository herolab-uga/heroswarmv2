// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/sensor_enable__rosidl_typesupport_fastrtps_cpp.hpp"
#include "robot_msgs/msg/detail/sensor_enable__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace robot_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_msgs
cdr_serialize(
  const robot_msgs::msg::SensorEnable & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: environment
  cdr << (ros_message.environment ? true : false);
  // Member: imu
  cdr << (ros_message.imu ? true : false);
  // Member: light
  cdr << (ros_message.light ? true : false);
  // Member: proximity
  cdr << (ros_message.proximity ? true : false);
  // Member: mic
  cdr << (ros_message.mic ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_msgs::msg::SensorEnable & ros_message)
{
  // Member: environment
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.environment = tmp ? true : false;
  }

  // Member: imu
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.imu = tmp ? true : false;
  }

  // Member: light
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.light = tmp ? true : false;
  }

  // Member: proximity
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.proximity = tmp ? true : false;
  }

  // Member: mic
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.mic = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_msgs
get_serialized_size(
  const robot_msgs::msg::SensorEnable & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: environment
  {
    size_t item_size = sizeof(ros_message.environment);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: imu
  {
    size_t item_size = sizeof(ros_message.imu);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: light
  {
    size_t item_size = sizeof(ros_message.light);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: proximity
  {
    size_t item_size = sizeof(ros_message.proximity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mic
  {
    size_t item_size = sizeof(ros_message.mic);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_msgs
max_serialized_size_SensorEnable(
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


  // Member: environment
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: imu
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: light
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: proximity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mic
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
    using DataType = robot_msgs::msg::SensorEnable;
    is_plain =
      (
      offsetof(DataType, mic) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SensorEnable__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const robot_msgs::msg::SensorEnable *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SensorEnable__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<robot_msgs::msg::SensorEnable *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SensorEnable__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const robot_msgs::msg::SensorEnable *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SensorEnable__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SensorEnable(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SensorEnable__callbacks = {
  "robot_msgs::msg",
  "SensorEnable",
  _SensorEnable__cdr_serialize,
  _SensorEnable__cdr_deserialize,
  _SensorEnable__get_serialized_size,
  _SensorEnable__max_serialized_size
};

static rosidl_message_type_support_t _SensorEnable__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SensorEnable__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robot_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_robot_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_msgs::msg::SensorEnable>()
{
  return &robot_msgs::msg::typesupport_fastrtps_cpp::_SensorEnable__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_msgs, msg, SensorEnable)() {
  return &robot_msgs::msg::typesupport_fastrtps_cpp::_SensorEnable__handle;
}

#ifdef __cplusplus
}
#endif
