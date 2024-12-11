// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SensorEnable in the package robot_msgs.
typedef struct robot_msgs__msg__SensorEnable
{
  bool environment;
  bool imu;
  bool light;
  bool proximity;
  bool mic;
} robot_msgs__msg__SensorEnable;

// Struct for a sequence of robot_msgs__msg__SensorEnable.
typedef struct robot_msgs__msg__SensorEnable__Sequence
{
  robot_msgs__msg__SensorEnable * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__SensorEnable__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_H_
