// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_msgs:srv/ReleaseCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__FUNCTIONS_H_
#define ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robot_msgs/srv/detail/release_charger__struct.h"

/// Initialize srv/ReleaseCharger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_msgs__srv__ReleaseCharger_Request
 * )) before or use
 * robot_msgs__srv__ReleaseCharger_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__init(robot_msgs__srv__ReleaseCharger_Request * msg);

/// Finalize srv/ReleaseCharger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Request__fini(robot_msgs__srv__ReleaseCharger_Request * msg);

/// Create srv/ReleaseCharger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_msgs__srv__ReleaseCharger_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__srv__ReleaseCharger_Request *
robot_msgs__srv__ReleaseCharger_Request__create();

/// Destroy srv/ReleaseCharger message.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Request__destroy(robot_msgs__srv__ReleaseCharger_Request * msg);

/// Check for srv/ReleaseCharger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__are_equal(const robot_msgs__srv__ReleaseCharger_Request * lhs, const robot_msgs__srv__ReleaseCharger_Request * rhs);

/// Copy a srv/ReleaseCharger message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__copy(
  const robot_msgs__srv__ReleaseCharger_Request * input,
  robot_msgs__srv__ReleaseCharger_Request * output);

/// Initialize array of srv/ReleaseCharger messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_msgs__srv__ReleaseCharger_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__Sequence__init(robot_msgs__srv__ReleaseCharger_Request__Sequence * array, size_t size);

/// Finalize array of srv/ReleaseCharger messages.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Request__Sequence__fini(robot_msgs__srv__ReleaseCharger_Request__Sequence * array);

/// Create array of srv/ReleaseCharger messages.
/**
 * It allocates the memory for the array and calls
 * robot_msgs__srv__ReleaseCharger_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__srv__ReleaseCharger_Request__Sequence *
robot_msgs__srv__ReleaseCharger_Request__Sequence__create(size_t size);

/// Destroy array of srv/ReleaseCharger messages.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Request__Sequence__destroy(robot_msgs__srv__ReleaseCharger_Request__Sequence * array);

/// Check for srv/ReleaseCharger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__Sequence__are_equal(const robot_msgs__srv__ReleaseCharger_Request__Sequence * lhs, const robot_msgs__srv__ReleaseCharger_Request__Sequence * rhs);

/// Copy an array of srv/ReleaseCharger messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Request__Sequence__copy(
  const robot_msgs__srv__ReleaseCharger_Request__Sequence * input,
  robot_msgs__srv__ReleaseCharger_Request__Sequence * output);

/// Initialize srv/ReleaseCharger message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_msgs__srv__ReleaseCharger_Response
 * )) before or use
 * robot_msgs__srv__ReleaseCharger_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__init(robot_msgs__srv__ReleaseCharger_Response * msg);

/// Finalize srv/ReleaseCharger message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Response__fini(robot_msgs__srv__ReleaseCharger_Response * msg);

/// Create srv/ReleaseCharger message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_msgs__srv__ReleaseCharger_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__srv__ReleaseCharger_Response *
robot_msgs__srv__ReleaseCharger_Response__create();

/// Destroy srv/ReleaseCharger message.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Response__destroy(robot_msgs__srv__ReleaseCharger_Response * msg);

/// Check for srv/ReleaseCharger message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__are_equal(const robot_msgs__srv__ReleaseCharger_Response * lhs, const robot_msgs__srv__ReleaseCharger_Response * rhs);

/// Copy a srv/ReleaseCharger message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__copy(
  const robot_msgs__srv__ReleaseCharger_Response * input,
  robot_msgs__srv__ReleaseCharger_Response * output);

/// Initialize array of srv/ReleaseCharger messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_msgs__srv__ReleaseCharger_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__Sequence__init(robot_msgs__srv__ReleaseCharger_Response__Sequence * array, size_t size);

/// Finalize array of srv/ReleaseCharger messages.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Response__Sequence__fini(robot_msgs__srv__ReleaseCharger_Response__Sequence * array);

/// Create array of srv/ReleaseCharger messages.
/**
 * It allocates the memory for the array and calls
 * robot_msgs__srv__ReleaseCharger_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__srv__ReleaseCharger_Response__Sequence *
robot_msgs__srv__ReleaseCharger_Response__Sequence__create(size_t size);

/// Destroy array of srv/ReleaseCharger messages.
/**
 * It calls
 * robot_msgs__srv__ReleaseCharger_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__srv__ReleaseCharger_Response__Sequence__destroy(robot_msgs__srv__ReleaseCharger_Response__Sequence * array);

/// Check for srv/ReleaseCharger message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__Sequence__are_equal(const robot_msgs__srv__ReleaseCharger_Response__Sequence * lhs, const robot_msgs__srv__ReleaseCharger_Response__Sequence * rhs);

/// Copy an array of srv/ReleaseCharger messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__srv__ReleaseCharger_Response__Sequence__copy(
  const robot_msgs__srv__ReleaseCharger_Response__Sequence * input,
  robot_msgs__srv__ReleaseCharger_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__FUNCTIONS_H_
