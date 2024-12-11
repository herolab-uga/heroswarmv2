// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BatteryStatus in the package robot_msgs.
typedef struct robot_msgs__msg__BatteryStatus
{
  float voltage;
  float soc;
  bool charging;
} robot_msgs__msg__BatteryStatus;

// Struct for a sequence of robot_msgs__msg__BatteryStatus.
typedef struct robot_msgs__msg__BatteryStatus__Sequence
{
  robot_msgs__msg__BatteryStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__BatteryStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_H_
