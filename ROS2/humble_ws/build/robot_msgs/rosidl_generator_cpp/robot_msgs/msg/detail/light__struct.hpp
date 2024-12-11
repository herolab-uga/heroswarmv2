// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__Light __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__Light __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Light_
{
  using Type = Light_<ContainerAllocator>;

  explicit Light_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gesture = 0l;
    }
  }

  explicit Light_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gesture = 0l;
    }
  }

  // field types and members
  using _rgbw_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _rgbw_type rgbw;
  using _gesture_type =
    int32_t;
  _gesture_type gesture;

  // setters for named parameter idiom
  Type & set__rgbw(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->rgbw = _arg;
    return *this;
  }
  Type & set__gesture(
    const int32_t & _arg)
  {
    this->gesture = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::Light_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::Light_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::Light_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::Light_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::Light_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::Light_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::Light_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::Light_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::Light_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::Light_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__Light
    std::shared_ptr<robot_msgs::msg::Light_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__Light
    std::shared_ptr<robot_msgs::msg::Light_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Light_ & other) const
  {
    if (this->rgbw != other.rgbw) {
      return false;
    }
    if (this->gesture != other.gesture) {
      return false;
    }
    return true;
  }
  bool operator!=(const Light_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Light_

// alias to use template instance with default allocator
using Light =
  robot_msgs::msg::Light_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__LIGHT__STRUCT_HPP_
