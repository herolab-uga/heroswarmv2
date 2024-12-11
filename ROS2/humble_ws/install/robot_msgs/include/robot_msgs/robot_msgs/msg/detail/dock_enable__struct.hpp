// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/DockEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__DockEnable __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__DockEnable __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DockEnable_
{
  using Type = DockEnable_<ContainerAllocator>;

  explicit DockEnable_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dock1 = 0;
      this->dock2 = 0;
      this->dock3 = 0;
      this->dock4 = 0;
    }
  }

  explicit DockEnable_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dock1 = 0;
      this->dock2 = 0;
      this->dock3 = 0;
      this->dock4 = 0;
    }
  }

  // field types and members
  using _dock1_type =
    uint8_t;
  _dock1_type dock1;
  using _dock2_type =
    uint8_t;
  _dock2_type dock2;
  using _dock3_type =
    uint8_t;
  _dock3_type dock3;
  using _dock4_type =
    uint8_t;
  _dock4_type dock4;

  // setters for named parameter idiom
  Type & set__dock1(
    const uint8_t & _arg)
  {
    this->dock1 = _arg;
    return *this;
  }
  Type & set__dock2(
    const uint8_t & _arg)
  {
    this->dock2 = _arg;
    return *this;
  }
  Type & set__dock3(
    const uint8_t & _arg)
  {
    this->dock3 = _arg;
    return *this;
  }
  Type & set__dock4(
    const uint8_t & _arg)
  {
    this->dock4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::DockEnable_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::DockEnable_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::DockEnable_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::DockEnable_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__DockEnable
    std::shared_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__DockEnable
    std::shared_ptr<robot_msgs::msg::DockEnable_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DockEnable_ & other) const
  {
    if (this->dock1 != other.dock1) {
      return false;
    }
    if (this->dock2 != other.dock2) {
      return false;
    }
    if (this->dock3 != other.dock3) {
      return false;
    }
    if (this->dock4 != other.dock4) {
      return false;
    }
    return true;
  }
  bool operator!=(const DockEnable_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DockEnable_

// alias to use template instance with default allocator
using DockEnable =
  robot_msgs::msg::DockEnable_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__DOCK_ENABLE__STRUCT_HPP_
