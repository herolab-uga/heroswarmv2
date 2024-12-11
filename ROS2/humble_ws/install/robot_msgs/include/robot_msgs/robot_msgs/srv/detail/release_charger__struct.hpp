// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:srv/ReleaseCharger.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__STRUCT_HPP_
#define ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'id'
#include "std_msgs/msg/detail/u_int16__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__srv__ReleaseCharger_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__srv__ReleaseCharger_Request __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ReleaseCharger_Request_
{
  using Type = ReleaseCharger_Request_<ContainerAllocator>;

  explicit ReleaseCharger_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id(_init)
  {
    (void)_init;
  }

  explicit ReleaseCharger_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _id_type =
    std_msgs::msg::UInt16_<ContainerAllocator>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__id(
    const std_msgs::msg::UInt16_<ContainerAllocator> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__srv__ReleaseCharger_Request
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__srv__ReleaseCharger_Request
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReleaseCharger_Request_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReleaseCharger_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReleaseCharger_Request_

// alias to use template instance with default allocator
using ReleaseCharger_Request =
  robot_msgs::srv::ReleaseCharger_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_msgs


// Include directives for member types
// Member 'released'
#include "std_msgs/msg/detail/bool__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__srv__ReleaseCharger_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__srv__ReleaseCharger_Response __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ReleaseCharger_Response_
{
  using Type = ReleaseCharger_Response_<ContainerAllocator>;

  explicit ReleaseCharger_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : released(_init)
  {
    (void)_init;
  }

  explicit ReleaseCharger_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : released(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _released_type =
    std_msgs::msg::Bool_<ContainerAllocator>;
  _released_type released;

  // setters for named parameter idiom
  Type & set__released(
    const std_msgs::msg::Bool_<ContainerAllocator> & _arg)
  {
    this->released = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__srv__ReleaseCharger_Response
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__srv__ReleaseCharger_Response
    std::shared_ptr<robot_msgs::srv::ReleaseCharger_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReleaseCharger_Response_ & other) const
  {
    if (this->released != other.released) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReleaseCharger_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReleaseCharger_Response_

// alias to use template instance with default allocator
using ReleaseCharger_Response =
  robot_msgs::srv::ReleaseCharger_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_msgs

namespace robot_msgs
{

namespace srv
{

struct ReleaseCharger
{
  using Request = robot_msgs::srv::ReleaseCharger_Request;
  using Response = robot_msgs::srv::ReleaseCharger_Response;
};

}  // namespace srv

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__SRV__DETAIL__RELEASE_CHARGER__STRUCT_HPP_
