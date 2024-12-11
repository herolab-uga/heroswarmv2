// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/Environment.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__Environment __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__Environment __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Environment_
{
  using Type = Environment_<ContainerAllocator>;

  explicit Environment_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temp = 0.0;
      this->pressure = 0.0;
      this->humidity = 0.0;
      this->altitude = 0.0;
    }
  }

  explicit Environment_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->temp = 0.0;
      this->pressure = 0.0;
      this->humidity = 0.0;
      this->altitude = 0.0;
    }
  }

  // field types and members
  using _temp_type =
    double;
  _temp_type temp;
  using _pressure_type =
    double;
  _pressure_type pressure;
  using _humidity_type =
    double;
  _humidity_type humidity;
  using _altitude_type =
    double;
  _altitude_type altitude;

  // setters for named parameter idiom
  Type & set__temp(
    const double & _arg)
  {
    this->temp = _arg;
    return *this;
  }
  Type & set__pressure(
    const double & _arg)
  {
    this->pressure = _arg;
    return *this;
  }
  Type & set__humidity(
    const double & _arg)
  {
    this->humidity = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::Environment_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::Environment_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::Environment_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::Environment_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::Environment_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::Environment_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::Environment_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::Environment_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::Environment_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::Environment_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__Environment
    std::shared_ptr<robot_msgs::msg::Environment_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__Environment
    std::shared_ptr<robot_msgs::msg::Environment_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Environment_ & other) const
  {
    if (this->temp != other.temp) {
      return false;
    }
    if (this->pressure != other.pressure) {
      return false;
    }
    if (this->humidity != other.humidity) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    return true;
  }
  bool operator!=(const Environment_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Environment_

// alias to use template instance with default allocator
using Environment =
  robot_msgs::msg::Environment_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ENVIRONMENT__STRUCT_HPP_
