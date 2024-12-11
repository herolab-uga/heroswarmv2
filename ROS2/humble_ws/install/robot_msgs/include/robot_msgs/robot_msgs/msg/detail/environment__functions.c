// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/Environment.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/environment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_msgs__msg__Environment__init(robot_msgs__msg__Environment * msg)
{
  if (!msg) {
    return false;
  }
  // temp
  // pressure
  // humidity
  // altitude
  return true;
}

void
robot_msgs__msg__Environment__fini(robot_msgs__msg__Environment * msg)
{
  if (!msg) {
    return;
  }
  // temp
  // pressure
  // humidity
  // altitude
}

bool
robot_msgs__msg__Environment__are_equal(const robot_msgs__msg__Environment * lhs, const robot_msgs__msg__Environment * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // temp
  if (lhs->temp != rhs->temp) {
    return false;
  }
  // pressure
  if (lhs->pressure != rhs->pressure) {
    return false;
  }
  // humidity
  if (lhs->humidity != rhs->humidity) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__Environment__copy(
  const robot_msgs__msg__Environment * input,
  robot_msgs__msg__Environment * output)
{
  if (!input || !output) {
    return false;
  }
  // temp
  output->temp = input->temp;
  // pressure
  output->pressure = input->pressure;
  // humidity
  output->humidity = input->humidity;
  // altitude
  output->altitude = input->altitude;
  return true;
}

robot_msgs__msg__Environment *
robot_msgs__msg__Environment__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__Environment * msg = (robot_msgs__msg__Environment *)allocator.allocate(sizeof(robot_msgs__msg__Environment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__Environment));
  bool success = robot_msgs__msg__Environment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__Environment__destroy(robot_msgs__msg__Environment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__Environment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__Environment__Sequence__init(robot_msgs__msg__Environment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__Environment * data = NULL;

  if (size) {
    data = (robot_msgs__msg__Environment *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__Environment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__Environment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__Environment__fini(&data[i - 1]);
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
robot_msgs__msg__Environment__Sequence__fini(robot_msgs__msg__Environment__Sequence * array)
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
      robot_msgs__msg__Environment__fini(&array->data[i]);
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

robot_msgs__msg__Environment__Sequence *
robot_msgs__msg__Environment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__Environment__Sequence * array = (robot_msgs__msg__Environment__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__Environment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__Environment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__Environment__Sequence__destroy(robot_msgs__msg__Environment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__Environment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__Environment__Sequence__are_equal(const robot_msgs__msg__Environment__Sequence * lhs, const robot_msgs__msg__Environment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__Environment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__Environment__Sequence__copy(
  const robot_msgs__msg__Environment__Sequence * input,
  robot_msgs__msg__Environment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__Environment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__Environment * data =
      (robot_msgs__msg__Environment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__Environment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__Environment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__Environment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
