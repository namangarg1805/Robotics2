// Generated by gencpp from file gazebo_ros_link_attacher/GripperResponse.msg
// DO NOT EDIT!


#ifndef GAZEBO_ROS_LINK_ATTACHER_MESSAGE_GRIPPERRESPONSE_H
#define GAZEBO_ROS_LINK_ATTACHER_MESSAGE_GRIPPERRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gazebo_ros_link_attacher
{
template <class ContainerAllocator>
struct GripperResponse_
{
  typedef GripperResponse_<ContainerAllocator> Type;

  GripperResponse_()
    : result(false)  {
    }
  GripperResponse_(const ContainerAllocator& _alloc)
    : result(false)  {
  (void)_alloc;
    }



   typedef uint8_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GripperResponse_

typedef ::gazebo_ros_link_attacher::GripperResponse_<std::allocator<void> > GripperResponse;

typedef boost::shared_ptr< ::gazebo_ros_link_attacher::GripperResponse > GripperResponsePtr;
typedef boost::shared_ptr< ::gazebo_ros_link_attacher::GripperResponse const> GripperResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator1> & lhs, const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator1> & lhs, const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gazebo_ros_link_attacher

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gazebo_ros_link_attacher/GripperResponse";
  }

  static const char* value(const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool result\n"
;
  }

  static const char* value(const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GripperResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gazebo_ros_link_attacher::GripperResponse_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GAZEBO_ROS_LINK_ATTACHER_MESSAGE_GRIPPERRESPONSE_H
