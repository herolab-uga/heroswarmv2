// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__BatteryStatus __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__BatteryStatus __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BatteryStatus_
{
  using Type = BatteryStatus_<ContainerAllocator>;

  explicit BatteryStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->soc = 0.0f;
      this->charging = false;
    }
  }

  explicit BatteryStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->soc = 0.0f;
      this->charging = false;
    }
  }

  // field types and members
  using _voltage_type =
    float;
  _voltage_type voltage;
  using _soc_type =
    float;
  _soc_type soc;
  using _charging_type =
    bool;
  _charging_type charging;

  // setters for named parameter idiom
  Type & set__voltage(
    const float & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__soc(
    const float & _arg)
  {
    this->soc = _arg;
    return *this;
  }
  Type & set__charging(
    const bool & _arg)
  {
    this->charging = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::BatteryStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::BatteryStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::BatteryStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::BatteryStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__BatteryStatus
    std::shared_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__BatteryStatus
    std::shared_ptr<robot_msgs::msg::BatteryStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BatteryStatus_ & other) const
  {
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->soc != other.soc) {
      return false;
    }
    if (this->charging != other.charging) {
      return false;
    }
    return true;
  }
  bool operator!=(const BatteryStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BatteryStatus_

// alias to use template instance with default allocator
using BatteryStatus =
  robot_msgs::msg::BatteryStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__BATTERY_STATUS__STRUCT_HPP_
