// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/SensorEnable.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__SensorEnable __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__SensorEnable __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorEnable_
{
  using Type = SensorEnable_<ContainerAllocator>;

  explicit SensorEnable_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->environment = false;
      this->imu = false;
      this->light = false;
      this->proximity = false;
      this->mic = false;
    }
  }

  explicit SensorEnable_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->environment = false;
      this->imu = false;
      this->light = false;
      this->proximity = false;
      this->mic = false;
    }
  }

  // field types and members
  using _environment_type =
    bool;
  _environment_type environment;
  using _imu_type =
    bool;
  _imu_type imu;
  using _light_type =
    bool;
  _light_type light;
  using _proximity_type =
    bool;
  _proximity_type proximity;
  using _mic_type =
    bool;
  _mic_type mic;

  // setters for named parameter idiom
  Type & set__environment(
    const bool & _arg)
  {
    this->environment = _arg;
    return *this;
  }
  Type & set__imu(
    const bool & _arg)
  {
    this->imu = _arg;
    return *this;
  }
  Type & set__light(
    const bool & _arg)
  {
    this->light = _arg;
    return *this;
  }
  Type & set__proximity(
    const bool & _arg)
  {
    this->proximity = _arg;
    return *this;
  }
  Type & set__mic(
    const bool & _arg)
  {
    this->mic = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::SensorEnable_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::SensorEnable_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::SensorEnable_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::SensorEnable_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__SensorEnable
    std::shared_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__SensorEnable
    std::shared_ptr<robot_msgs::msg::SensorEnable_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorEnable_ & other) const
  {
    if (this->environment != other.environment) {
      return false;
    }
    if (this->imu != other.imu) {
      return false;
    }
    if (this->light != other.light) {
      return false;
    }
    if (this->proximity != other.proximity) {
      return false;
    }
    if (this->mic != other.mic) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorEnable_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorEnable_

// alias to use template instance with default allocator
using SensorEnable =
  robot_msgs::msg::SensorEnable_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__SENSOR_ENABLE__STRUCT_HPP_
