// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/DistanceSensor.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DistanceSensor in the package robot_msgs.
typedef struct robot_msgs__msg__DistanceSensor
{
  int32_t sensor1;
  int32_t sensor2;
  int32_t sensor3;
  int32_t sensor4;
} robot_msgs__msg__DistanceSensor;

// Struct for a sequence of robot_msgs__msg__DistanceSensor.
typedef struct robot_msgs__msg__DistanceSensor__Sequence
{
  robot_msgs__msg__DistanceSensor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__DistanceSensor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_H_
