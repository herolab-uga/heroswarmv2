// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/sensor_enable__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_msgs__msg__SensorEnable__init(robot_msgs__msg__SensorEnable * msg)
{
  if (!msg) {
    return false;
  }
  // environment
  // imu
  // light
  // proximity
  // mic
  return true;
}

void
robot_msgs__msg__SensorEnable__fini(robot_msgs__msg__SensorEnable * msg)
{
  if (!msg) {
    return;
  }
  // environment
  // imu
  // light
  // proximity
  // mic
}

bool
robot_msgs__msg__SensorEnable__are_equal(const robot_msgs__msg__SensorEnable * lhs, const robot_msgs__msg__SensorEnable * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // environment
  if (lhs->environment != rhs->environment) {
    return false;
  }
  // imu
  if (lhs->imu != rhs->imu) {
    return false;
  }
  // light
  if (lhs->light != rhs->light) {
    return false;
  }
  // proximity
  if (lhs->proximity != rhs->proximity) {
    return false;
  }
  // mic
  if (lhs->mic != rhs->mic) {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__SensorEnable__copy(
  const robot_msgs__msg__SensorEnable * input,
  robot_msgs__msg__SensorEnable * output)
{
  if (!input || !output) {
    return false;
  }
  // environment
  output->environment = input->environment;
  // imu
  output->imu = input->imu;
  // light
  output->light = input->light;
  // proximity
  output->proximity = input->proximity;
  // mic
  output->mic = input->mic;
  return true;
}

robot_msgs__msg__SensorEnable *
robot_msgs__msg__SensorEnable__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__SensorEnable * msg = (robot_msgs__msg__SensorEnable *)allocator.allocate(sizeof(robot_msgs__msg__SensorEnable), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__SensorEnable));
  bool success = robot_msgs__msg__SensorEnable__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__SensorEnable__destroy(robot_msgs__msg__SensorEnable * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__SensorEnable__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__SensorEnable__Sequence__init(robot_msgs__msg__SensorEnable__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__SensorEnable * data = NULL;

  if (size) {
    data = (robot_msgs__msg__SensorEnable *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__SensorEnable), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__SensorEnable__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__SensorEnable__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_msgs__msg__SensorEnable__Sequence__fini(robot_msgs__msg__SensorEnable__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_msgs__msg__SensorEnable__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_msgs__msg__SensorEnable__Sequence *
robot_msgs__msg__SensorEnable__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__SensorEnable__Sequence * array = (robot_msgs__msg__SensorEnable__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__SensorEnable__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__SensorEnable__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__SensorEnable__Sequence__destroy(robot_msgs__msg__SensorEnable__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__SensorEnable__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__SensorEnable__Sequence__are_equal(const robot_msgs__msg__SensorEnable__Sequence * lhs, const robot_msgs__msg__SensorEnable__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__SensorEnable__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__SensorEnable__Sequence__copy(
  const robot_msgs__msg__SensorEnable__Sequence * input,
  robot_msgs__msg__SensorEnable__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__SensorEnable);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__SensorEnable * data =
      (robot_msgs__msg__SensorEnable *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__SensorEnable__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__SensorEnable__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__SensorEnable__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
