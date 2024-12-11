// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_pos'
#include "nav_msgs/msg/detail/odometry__struct.h"

/// Struct defined in msg/RobotPos in the package robot_msgs.
typedef struct robot_msgs__msg__RobotPos
{
  nav_msgs__msg__Odometry__Sequence robot_pos;
} robot_msgs__msg__RobotPos;

// Struct for a sequence of robot_msgs__msg__RobotPos.
typedef struct robot_msgs__msg__RobotPos__Sequence
{
  robot_msgs__msg__RobotPos * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__RobotPos__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_H_
