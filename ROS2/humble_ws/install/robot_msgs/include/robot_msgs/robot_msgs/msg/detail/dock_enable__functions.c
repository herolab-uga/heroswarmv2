// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice
#include "robot_msgs/msg/detail/dock_enable__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_msgs__msg__DockEnable__init(robot_msgs__msg__DockEnable * msg)
{
  if (!msg) {
    return false;
  }
  // dock1
  // dock2
  // dock3
  // dock4
  return true;
}

void
robot_msgs__msg__DockEnable__fini(robot_msgs__msg__DockEnable * msg)
{
  if (!msg) {
    return;
  }
  // dock1
  // dock2
  // dock3
  // dock4
}

bool
robot_msgs__msg__DockEnable__are_equal(const robot_msgs__msg__DockEnable * lhs, const robot_msgs__msg__DockEnable * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // dock1
  if (lhs->dock1 != rhs->dock1) {
    return false;
  }
  // dock2
  if (lhs->dock2 != rhs->dock2) {
    return false;
  }
  // dock3
  if (lhs->dock3 != rhs->dock3) {
    return false;
  }
  // dock4
  if (lhs->dock4 != rhs->dock4) {
    return false;
  }
  return true;
}

bool
robot_msgs__msg__DockEnable__copy(
  const robot_msgs__msg__DockEnable * input,
  robot_msgs__msg__DockEnable * output)
{
  if (!input || !output) {
    return false;
  }
  // dock1
  output->dock1 = input->dock1;
  // dock2
  output->dock2 = input->dock2;
  // dock3
  output->dock3 = input->dock3;
  // dock4
  output->dock4 = input->dock4;
  return true;
}

robot_msgs__msg__DockEnable *
robot_msgs__msg__DockEnable__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__DockEnable * msg = (robot_msgs__msg__DockEnable *)allocator.allocate(sizeof(robot_msgs__msg__DockEnable), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msgs__msg__DockEnable));
  bool success = robot_msgs__msg__DockEnable__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msgs__msg__DockEnable__destroy(robot_msgs__msg__DockEnable * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msgs__msg__DockEnable__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msgs__msg__DockEnable__Sequence__init(robot_msgs__msg__DockEnable__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__DockEnable * data = NULL;

  if (size) {
    data = (robot_msgs__msg__DockEnable *)allocator.zero_allocate(size, sizeof(robot_msgs__msg__DockEnable), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msgs__msg__DockEnable__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msgs__msg__DockEnable__fini(&data[i - 1]);
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
robot_msgs__msg__DockEnable__Sequence__fini(robot_msgs__msg__DockEnable__Sequence * array)
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
      robot_msgs__msg__DockEnable__fini(&array->data[i]);
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

robot_msgs__msg__DockEnable__Sequence *
robot_msgs__msg__DockEnable__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msgs__msg__DockEnable__Sequence * array = (robot_msgs__msg__DockEnable__Sequence *)allocator.allocate(sizeof(robot_msgs__msg__DockEnable__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msgs__msg__DockEnable__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msgs__msg__DockEnable__Sequence__destroy(robot_msgs__msg__DockEnable__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msgs__msg__DockEnable__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msgs__msg__DockEnable__Sequence__are_equal(const robot_msgs__msg__DockEnable__Sequence * lhs, const robot_msgs__msg__DockEnable__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msgs__msg__DockEnable__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msgs__msg__DockEnable__Sequence__copy(
  const robot_msgs__msg__DockEnable__Sequence * input,
  robot_msgs__msg__DockEnable__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msgs__msg__DockEnable);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msgs__msg__DockEnable * data =
      (robot_msgs__msg__DockEnable *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msgs__msg__DockEnable__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msgs__msg__DockEnable__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msgs__msg__DockEnable__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
