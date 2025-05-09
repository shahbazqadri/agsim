// Generated by gencpp from file motoman_msgs/CmdJointTrajectoryExResponse.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXRESPONSE_H
#define MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <industrial_msgs/ServiceReturnCode.h>

namespace motoman_msgs
{
template <class ContainerAllocator>
struct CmdJointTrajectoryExResponse_
{
  typedef CmdJointTrajectoryExResponse_<ContainerAllocator> Type;

  CmdJointTrajectoryExResponse_()
    : code()  {
    }
  CmdJointTrajectoryExResponse_(const ContainerAllocator& _alloc)
    : code(_alloc)  {
  (void)_alloc;
    }



   typedef  ::industrial_msgs::ServiceReturnCode_<ContainerAllocator>  _code_type;
  _code_type code;





  typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CmdJointTrajectoryExResponse_

typedef ::motoman_msgs::CmdJointTrajectoryExResponse_<std::allocator<void> > CmdJointTrajectoryExResponse;

typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExResponse > CmdJointTrajectoryExResponsePtr;
typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExResponse const> CmdJointTrajectoryExResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'industrial_msgs': ['/opt/ros/kinetic/share/industrial_msgs/cmake/../msg'], 'motoman_msgs': ['/home/sawyercontrol/Desktop/robotcontrolpkg/src/motoman/src/motoman/motoman_msgs/msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "50b1f38f75f5677e5692f3b3e7e1ea48";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x50b1f38f75f5677eULL;
  static const uint64_t static_value2 = 0x5692f3b3e7e1ea48ULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/CmdJointTrajectoryExResponse";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "industrial_msgs/ServiceReturnCode code\n\
\n\
\n\
================================================================================\n\
MSG: industrial_msgs/ServiceReturnCode\n\
# Service return codes for simple requests.  All ROS-Industrial service\n\
# replies are required to have a return code indicating success or failure\n\
# Specific return codes for different failure should be negative.\n\
int8 val\n\
\n\
int8 SUCCESS = 1\n\
int8 FAILURE = -1\n\
\n\
";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CmdJointTrajectoryExResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::CmdJointTrajectoryExResponse_<ContainerAllocator>& v)
  {
    s << indent << "code: ";
    s << std::endl;
    Printer< ::industrial_msgs::ServiceReturnCode_<ContainerAllocator> >::stream(s, indent + "  ", v.code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXRESPONSE_H
