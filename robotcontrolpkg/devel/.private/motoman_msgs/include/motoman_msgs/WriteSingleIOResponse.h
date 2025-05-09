// Generated by gencpp from file motoman_msgs/WriteSingleIOResponse.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_WRITESINGLEIORESPONSE_H
#define MOTOMAN_MSGS_MESSAGE_WRITESINGLEIORESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motoman_msgs
{
template <class ContainerAllocator>
struct WriteSingleIOResponse_
{
  typedef WriteSingleIOResponse_<ContainerAllocator> Type;

  WriteSingleIOResponse_()
    : message()
    , success(false)  {
    }
  WriteSingleIOResponse_(const ContainerAllocator& _alloc)
    : message(_alloc)
    , success(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;

   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> const> ConstPtr;

}; // struct WriteSingleIOResponse_

typedef ::motoman_msgs::WriteSingleIOResponse_<std::allocator<void> > WriteSingleIOResponse;

typedef boost::shared_ptr< ::motoman_msgs::WriteSingleIOResponse > WriteSingleIOResponsePtr;
typedef boost::shared_ptr< ::motoman_msgs::WriteSingleIOResponse const> WriteSingleIOResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'industrial_msgs': ['/opt/ros/kinetic/share/industrial_msgs/cmake/../msg'], 'motoman_msgs': ['/home/sawyercontrol/Desktop/robotcontrolpkg/src/motoman/src/motoman/motoman_msgs/msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9bf829f07d795d3f9e541a07897da2c4";
  }

  static const char* value(const ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9bf829f07d795d3fULL;
  static const uint64_t static_value2 = 0x9e541a07897da2c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/WriteSingleIOResponse";
  }

  static const char* value(const ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string message\n\
bool success\n\
\n\
";
  }

  static const char* value(const ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WriteSingleIOResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::WriteSingleIOResponse_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_WRITESINGLEIORESPONSE_H
