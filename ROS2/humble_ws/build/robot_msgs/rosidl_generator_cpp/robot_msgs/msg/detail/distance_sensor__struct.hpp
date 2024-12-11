// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/DistanceSensor.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__DistanceSensor __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__DistanceSensor __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DistanceSensor_
{
  using Type = DistanceSensor_<ContainerAllocator>;

  explicit DistanceSensor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor1 = 0l;
      this->sensor2 = 0l;
      this->sensor3 = 0l;
      this->sensor4 = 0l;
    }
  }

  explicit DistanceSensor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sensor1 = 0l;
      this->sensor2 = 0l;
      this->sensor3 = 0l;
      this->sensor4 = 0l;
    }
  }

  // field types and members
  using _sensor1_type =
    int32_t;
  _sensor1_type sensor1;
  using _sensor2_type =
    int32_t;
  _sensor2_type sensor2;
  using _sensor3_type =
    int32_t;
  _sensor3_type sensor3;
  using _sensor4_type =
    int32_t;
  _sensor4_type sensor4;

  // setters for named parameter idiom
  Type & set__sensor1(
    const int32_t & _arg)
  {
    this->sensor1 = _arg;
    return *this;
  }
  Type & set__sensor2(
    const int32_t & _arg)
  {
    this->sensor2 = _arg;
    return *this;
  }
  Type & set__sensor3(
    const int32_t & _arg)
  {
    this->sensor3 = _arg;
    return *this;
  }
  Type & set__sensor4(
    const int32_t & _arg)
  {
    this->sensor4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::DistanceSensor_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::DistanceSensor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::DistanceSensor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::DistanceSensor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__DistanceSensor
    std::shared_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__DistanceSensor
    std::shared_ptr<robot_msgs::msg::DistanceSensor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DistanceSensor_ & other) const
  {
    if (this->sensor1 != other.sensor1) {
      return false;
    }
    if (this->sensor2 != other.sensor2) {
      return false;
    }
    if (this->sensor3 != other.sensor3) {
      return false;
    }
    if (this->sensor4 != other.sensor4) {
      return false;
    }
    return true;
  }
  bool operator!=(const DistanceSensor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DistanceSensor_

// alias to use template instance with default allocator
using DistanceSensor =
  robot_msgs::msg::DistanceSensor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__DISTANCE_SENSOR__STRUCT_HPP_
