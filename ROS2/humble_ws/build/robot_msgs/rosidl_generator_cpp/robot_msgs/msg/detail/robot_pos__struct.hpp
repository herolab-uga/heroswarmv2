// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/RobotPos.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'robot_pos'
#include "nav_msgs/msg/detail/odometry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__RobotPos __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__RobotPos __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPos_
{
  using Type = RobotPos_<ContainerAllocator>;

  explicit RobotPos_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RobotPos_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _robot_pos_type =
    std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>>;
  _robot_pos_type robot_pos;

  // setters for named parameter idiom
  Type & set__robot_pos(
    const std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>> & _arg)
  {
    this->robot_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::RobotPos_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::RobotPos_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::RobotPos_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::RobotPos_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__RobotPos
    std::shared_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__RobotPos
    std::shared_ptr<robot_msgs::msg::RobotPos_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPos_ & other) const
  {
    if (this->robot_pos != other.robot_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPos_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPos_

// alias to use template instance with default allocator
using RobotPos =
  robot_msgs::msg::RobotPos_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ROBOT_POS__STRUCT_HPP_
