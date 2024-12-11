// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_msgs:srv/GetCharger.idl
// generated code does not contain a copyright notice
#include "robot_msgs/srv/detail/get_charger__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_msgs/srv/detail/get_charger__struct.h"
#include "robot_msgs/srv/detail/get_charger__functions.h"
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

#include "std_msgs/msg/detail/string__functions.h"  // name

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t get_serialized_size_std_msgs__msg__String(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t max_serialized_size_std_msgs__msg__String(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, String)();


using _GetCharger_Request__ros_msg_type = robot_msgs__srv__GetCharger_Request;

static bool _GetCharger_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetCharger_Request__ros_msg_type * ros_message = static_cast<const _GetCharger_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->name, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetCharger_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetCharger_Request__ros_msg_type * ros_message = static_cast<_GetCharger_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: name
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, String
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->name))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_robot_msgs__srv__GetCharger_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetCharger_Request__ros_msg_type * ros_message = static_cast<const _GetCharger_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name name

  current_alignment += get_serialized_size_std_msgs__msg__String(
    &(ros_message->name), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetCharger_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_msgs__srv__GetCharger_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_robot_msgs__srv__GetCharger_Request(
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

  // member: name
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__String(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_msgs__srv__GetCharger_Request;
    is_plain =
      (
      offsetof(DataType, name) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetCharger_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_msgs__srv__GetCharger_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetCharger_Request = {
  "robot_msgs::srv",
  "GetCharger_Request",
  _GetCharger_Request__cdr_serialize,
  _GetCharger_Request__cdr_deserialize,
  _GetCharger_Request__get_serialized_size,
  _GetCharger_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetCharger_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetCharger_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, GetCharger_Request)() {
  return &_GetCharger_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "robot_msgs/srv/detail/get_charger__struct.h"
// already included above
// #include "robot_msgs/srv/detail/get_charger__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "geometry_msgs/msg/detail/pose__functions.h"  // position
#include "std_msgs/msg/detail/u_int16__functions.h"  // id

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t get_serialized_size_geometry_msgs__msg__Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t max_serialized_size_geometry_msgs__msg__Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t get_serialized_size_std_msgs__msg__UInt16(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
size_t max_serialized_size_std_msgs__msg__UInt16(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_robot_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt16)();


using _GetCharger_Response__ros_msg_type = robot_msgs__srv__GetCharger_Response;

static bool _GetCharger_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetCharger_Response__ros_msg_type * ros_message = static_cast<const _GetCharger_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt16
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->id, cdr))
    {
      return false;
    }
  }

  // Field name: position
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->position, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetCharger_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetCharger_Response__ros_msg_type * ros_message = static_cast<_GetCharger_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, UInt16
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->id))
    {
      return false;
    }
  }

  // Field name: position
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->position))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_robot_msgs__srv__GetCharger_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetCharger_Response__ros_msg_type * ros_message = static_cast<const _GetCharger_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id

  current_alignment += get_serialized_size_std_msgs__msg__UInt16(
    &(ros_message->id), current_alignment);
  // field.name position

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose(
    &(ros_message->position), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetCharger_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_msgs__srv__GetCharger_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_robot_msgs__srv__GetCharger_Response(
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

  // member: id
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__UInt16(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: position
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = robot_msgs__srv__GetCharger_Response;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetCharger_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_robot_msgs__srv__GetCharger_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetCharger_Response = {
  "robot_msgs::srv",
  "GetCharger_Response",
  _GetCharger_Response__cdr_serialize,
  _GetCharger_Response__cdr_deserialize,
  _GetCharger_Response__get_serialized_size,
  _GetCharger_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetCharger_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetCharger_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, GetCharger_Response)() {
  return &_GetCharger_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_msgs/srv/get_charger.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetCharger__callbacks = {
  "robot_msgs::srv",
  "GetCharger",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, GetCharger_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, GetCharger_Response)(),
};

static rosidl_service_type_support_t GetCharger__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetCharger__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, srv, GetCharger)() {
  return &GetCharger__handle;
}

#if defined(__cplusplus)
}
#endif
