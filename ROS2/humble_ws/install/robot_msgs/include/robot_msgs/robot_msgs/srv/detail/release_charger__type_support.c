// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:srv/ReleaseCharger.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/srv/detail/release_charger__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/srv/detail/release_charger__functions.h"
#include "robot_msgs/srv/detail/release_charger__struct.h"


// Include directives for member types
// Member `id`
#include "std_msgs/msg/u_int16.h"
// Member `id`
#include "std_msgs/msg/detail/u_int16__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__srv__ReleaseCharger_Request__init(message_memory);
}

void robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_fini_function(void * message_memory)
{
  robot_msgs__srv__ReleaseCharger_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_member_array[1] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__srv__ReleaseCharger_Request, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_members = {
  "robot_msgs__srv",  // message namespace
  "ReleaseCharger_Request",  // message name
  1,  // number of fields
  sizeof(robot_msgs__srv__ReleaseCharger_Request),
  robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_member_array,  // message members
  robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_type_support_handle = {
  0,
  &robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Request)() {
  robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, UInt16)();
  if (!robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_type_support_handle.typesupport_identifier) {
    robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__srv__ReleaseCharger_Request__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_msgs/srv/detail/release_charger__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_msgs/srv/detail/release_charger__functions.h"
// already included above
// #include "robot_msgs/srv/detail/release_charger__struct.h"


// Include directives for member types
// Member `released`
#include "std_msgs/msg/bool.h"
// Member `released`
#include "std_msgs/msg/detail/bool__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__srv__ReleaseCharger_Response__init(message_memory);
}

void robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_fini_function(void * message_memory)
{
  robot_msgs__srv__ReleaseCharger_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_member_array[1] = {
  {
    "released",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__srv__ReleaseCharger_Response, released),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_members = {
  "robot_msgs__srv",  // message namespace
  "ReleaseCharger_Response",  // message name
  1,  // number of fields
  sizeof(robot_msgs__srv__ReleaseCharger_Response),
  robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_member_array,  // message members
  robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_type_support_handle = {
  0,
  &robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Response)() {
  robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Bool)();
  if (!robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_type_support_handle.typesupport_identifier) {
    robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__srv__ReleaseCharger_Response__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_msgs/srv/detail/release_charger__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_members = {
  "robot_msgs__srv",  // service namespace
  "ReleaseCharger",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_Request_message_type_support_handle,
  NULL  // response message
  // robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_type_support_handle = {
  0,
  &robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger)() {
  if (!robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_type_support_handle.typesupport_identifier) {
    robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, srv, ReleaseCharger_Response)()->data;
  }

  return &robot_msgs__srv__detail__release_charger__rosidl_typesupport_introspection_c__ReleaseCharger_service_type_support_handle;
}
