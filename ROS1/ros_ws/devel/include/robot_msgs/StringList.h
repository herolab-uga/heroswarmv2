// Generated by gencpp from file robot_msgs/StringList.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_STRINGLIST_H
#define ROBOT_MSGS_MESSAGE_STRINGLIST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace robot_msgs
{
template <class ContainerAllocator>
struct StringList_
{
  typedef StringList_<ContainerAllocator> Type;

  StringList_()
    : names()  {
    }
  StringList_(const ContainerAllocator& _alloc)
    : names(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::std_msgs::String_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::String_<ContainerAllocator> >::other >  _names_type;
  _names_type names;





  typedef boost::shared_ptr< ::robot_msgs::StringList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::StringList_<ContainerAllocator> const> ConstPtr;

}; // struct StringList_

typedef ::robot_msgs::StringList_<std::allocator<void> > StringList;

typedef boost::shared_ptr< ::robot_msgs::StringList > StringListPtr;
typedef boost::shared_ptr< ::robot_msgs::StringList const> StringListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::StringList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::StringList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_msgs::StringList_<ContainerAllocator1> & lhs, const ::robot_msgs::StringList_<ContainerAllocator2> & rhs)
{
  return lhs.names == rhs.names;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_msgs::StringList_<ContainerAllocator1> & lhs, const ::robot_msgs::StringList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::StringList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::StringList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::StringList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::StringList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::StringList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::StringList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::StringList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5334ccc36929d3443e2083f7204590bf";
  }

  static const char* value(const ::robot_msgs::StringList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5334ccc36929d344ULL;
  static const uint64_t static_value2 = 0x3e2083f7204590bfULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::StringList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/StringList";
  }

  static const char* value(const ::robot_msgs::StringList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::StringList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/String[] names\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::robot_msgs::StringList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::StringList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.names);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StringList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::StringList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::StringList_<ContainerAllocator>& v)
  {
    s << indent << "names[]" << std::endl;
    for (size_t i = 0; i < v.names.size(); ++i)
    {
      s << indent << "  names[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "    ", v.names[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_STRINGLIST_H
