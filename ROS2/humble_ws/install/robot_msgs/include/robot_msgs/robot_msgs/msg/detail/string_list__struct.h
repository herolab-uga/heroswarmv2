// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in msg/StringList in the package robot_msgs.
typedef struct robot_msgs__msg__StringList
{
  std_msgs__msg__String__Sequence data;
} robot_msgs__msg__StringList;

// Struct for a sequence of robot_msgs__msg__StringList.
typedef struct robot_msgs__msg__StringList__Sequence
{
  robot_msgs__msg__StringList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__StringList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_H_
