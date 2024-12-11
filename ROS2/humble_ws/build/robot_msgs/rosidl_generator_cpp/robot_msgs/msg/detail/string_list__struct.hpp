// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/StringList.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'data'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__StringList __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__StringList __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StringList_
{
  using Type = StringList_<ContainerAllocator>;

  explicit StringList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit StringList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<std_msgs::msg::String_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::String_<ContainerAllocator>>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<std_msgs::msg::String_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std_msgs::msg::String_<ContainerAllocator>>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::StringList_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::StringList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::StringList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::StringList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::StringList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::StringList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::StringList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::StringList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::StringList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::StringList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__StringList
    std::shared_ptr<robot_msgs::msg::StringList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__StringList
    std::shared_ptr<robot_msgs::msg::StringList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StringList_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const StringList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StringList_

// alias to use template instance with default allocator
using StringList =
  robot_msgs::msg::StringList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__STRING_LIST__STRUCT_HPP_
