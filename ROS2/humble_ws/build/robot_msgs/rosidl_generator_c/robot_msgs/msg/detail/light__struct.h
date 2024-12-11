// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'rgbw'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Light in the package robot_msgs.
typedef struct robot_msgs__msg__Light
{
  rosidl_runtime_c__int32__Sequence rgbw;
  int32_t gesture;
} robot_msgs__msg__Light;

// Struct for a sequence of robot_msgs__msg__Light.
typedef struct robot_msgs__msg__Light__Sequence
{
  robot_msgs__msg__Light * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__Light__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_H_
