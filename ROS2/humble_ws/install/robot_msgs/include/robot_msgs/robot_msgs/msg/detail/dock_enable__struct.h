// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DockEnable in the package robot_msgs.
typedef struct robot_msgs__msg__DockEnable
{
  uint8_t dock1;
  uint8_t dock2;
  uint8_t dock3;
  uint8_t dock4;
} robot_msgs__msg__DockEnable;

// Struct for a sequence of robot_msgs__msg__DockEnable.
typedef struct robot_msgs__msg__DockEnable__Sequence
{
  robot_msgs__msg__DockEnable * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__DockEnable__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_H_
