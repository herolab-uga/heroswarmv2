// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:srv/GetCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__STRUCT_H_
#define ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in srv/GetCharger in the package robot_msgs.
typedef struct robot_msgs__srv__GetCharger_Request
{
  std_msgs__msg__String name;
} robot_msgs__srv__GetCharger_Request;

// Struct for a sequence of robot_msgs__srv__GetCharger_Request.
typedef struct robot_msgs__srv__GetCharger_Request__Sequence
{
  robot_msgs__srv__GetCharger_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__srv__GetCharger_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'id'
#include "std_msgs/msg/detail/u_int16__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/GetCharger in the package robot_msgs.
typedef struct robot_msgs__srv__GetCharger_Response
{
  std_msgs__msg__UInt16 id;
  geometry_msgs__msg__Pose position;
} robot_msgs__srv__GetCharger_Response;

// Struct for a sequence of robot_msgs__srv__GetCharger_Response.
typedef struct robot_msgs__srv__GetCharger_Response__Sequence
{
  robot_msgs__srv__GetCharger_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__srv__GetCharger_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__SRV__DETAIL__GET_CHARGER__STRUCT_H_
