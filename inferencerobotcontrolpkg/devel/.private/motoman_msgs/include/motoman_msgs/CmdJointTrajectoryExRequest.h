// Generated by gencpp from file motoman_msgs/CmdJointTrajectoryExRequest.msg
// DO NOT EDIT!


#ifndef MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXREQUEST_H
#define MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <motoman_msgs/DynamicJointTrajectory.h>

namespace motoman_msgs
{
template <class ContainerAllocator>
struct CmdJointTrajectoryExRequest_
{
  typedef CmdJointTrajectoryExRequest_<ContainerAllocator> Type;

  CmdJointTrajectoryExRequest_()
    : trajectory()  {
    }
  CmdJointTrajectoryExRequest_(const ContainerAllocator& _alloc)
    : trajectory(_alloc)  {
  (void)_alloc;
    }



   typedef  ::motoman_msgs::DynamicJointTrajectory_<ContainerAllocator>  _trajectory_type;
  _trajectory_type trajectory;





  typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CmdJointTrajectoryExRequest_

typedef ::motoman_msgs::CmdJointTrajectoryExRequest_<std::allocator<void> > CmdJointTrajectoryExRequest;

typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExRequest > CmdJointTrajectoryExRequestPtr;
typedef boost::shared_ptr< ::motoman_msgs::CmdJointTrajectoryExRequest const> CmdJointTrajectoryExRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motoman_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'industrial_msgs': ['/opt/ros/kinetic/share/industrial_msgs/cmake/../msg'], 'motoman_msgs': ['/home/wmf-admin/Desktop/inferencerobotcontrolpkg/src/motoman/motoman_msgs/msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7896a81fd909fb14239085c546790e08";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7896a81fd909fb14ULL;
  static const uint64_t static_value2 = 0x239085c546790e08ULL;
};

template<class ContainerAllocator>
struct DataType< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motoman_msgs/CmdJointTrajectoryExRequest";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
motoman_msgs/DynamicJointTrajectory trajectory\n\
\n\
================================================================================\n\
MSG: motoman_msgs/DynamicJointTrajectory\n\
#length: true message/data length\n\
#header: \n\
#sequence:\n\
#num_groups: # of motion groups included in this message\n\
#group[]: DynamicJointPoint from DynamicJointPoint.msg\n\
\n\
Header header\n\
string[] joint_names\n\
DynamicJointPoint[] points\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: motoman_msgs/DynamicJointPoint\n\
# DynamicJointPoint\n\
#group: # length of this array must match num_groups\n\
#    id:   control-group ID for use on-controller\n\
#    num_joints: # of joints in this motion group\n\
#    valid_fields: #bit field for following items\n\
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n\
#    positions[]\n\
#    velocities[]\n\
#    accelerations[]\n\
#    effort[]\n\
#    time_from_start\n\
\n\
int16 num_groups\n\
DynamicJointsGroup[] groups\n\
\n\
================================================================================\n\
MSG: motoman_msgs/DynamicJointsGroup\n\
# DynamicJointsGroup\n\
#group: # length of this array must match num_groups\n\
#    id:   control-group ID for use on-controller\n\
#    num_joints: # of joints in this motion group\n\
#    valid_fields: #bit field for following items\n\
#    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.\n\
#    positions[]\n\
#    velocities[]\n\
#    accelerations[]\n\
#    effort[]\n\
#    time_from_start\n\
\n\
\n\
int16 group_number\n\
int16 num_joints\n\
int16 valid_fields\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
float64[] effort\n\
duration time_from_start\n\
";
  }

  static const char* value(const ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.trajectory);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CmdJointTrajectoryExRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motoman_msgs::CmdJointTrajectoryExRequest_<ContainerAllocator>& v)
  {
    s << indent << "trajectory: ";
    s << std::endl;
    Printer< ::motoman_msgs::DynamicJointTrajectory_<ContainerAllocator> >::stream(s, indent + "  ", v.trajectory);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOMAN_MSGS_MESSAGE_CMDJOINTTRAJECTORYEXREQUEST_H
