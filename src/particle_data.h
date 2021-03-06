// Generated by gencpp from file my_particle_filter/particle_data.msg
// DO NOT EDIT!


#ifndef MY_PARTICLE_FILTER_MESSAGE_PARTICLE_DATA_H
#define MY_PARTICLE_FILTER_MESSAGE_PARTICLE_DATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_particle_filter
{
template <class ContainerAllocator>
struct particle_data_
{
  typedef particle_data_<ContainerAllocator> Type;

  particle_data_()
    : x(0.0)
    , y(0.0)
    , v(0.0)
    , yaw_theta(0.0)
    , yaw_rate(0.0)  {
    }
  particle_data_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , v(0.0)
    , yaw_theta(0.0)
    , yaw_rate(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _v_type;
  _v_type v;

   typedef double _yaw_theta_type;
  _yaw_theta_type yaw_theta;

   typedef double _yaw_rate_type;
  _yaw_rate_type yaw_rate;





  typedef boost::shared_ptr< ::my_particle_filter::particle_data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_particle_filter::particle_data_<ContainerAllocator> const> ConstPtr;

}; // struct particle_data_

typedef ::my_particle_filter::particle_data_<std::allocator<void> > particle_data;

typedef boost::shared_ptr< ::my_particle_filter::particle_data > particle_dataPtr;
typedef boost::shared_ptr< ::my_particle_filter::particle_data const> particle_dataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_particle_filter::particle_data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_particle_filter::particle_data_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace my_particle_filter

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'my_particle_filter': ['/home/juchunyu/catkin_ws/src/my_particle_filter/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::my_particle_filter::particle_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_particle_filter::particle_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_particle_filter::particle_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_particle_filter::particle_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_particle_filter::particle_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_particle_filter::particle_data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_particle_filter::particle_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d360e0ec8763ee3b999b991e35bc513";
  }

  static const char* value(const ::my_particle_filter::particle_data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d360e0ec8763ee3ULL;
  static const uint64_t static_value2 = 0xb999b991e35bc513ULL;
};

template<class ContainerAllocator>
struct DataType< ::my_particle_filter::particle_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_particle_filter/particle_data";
  }

  static const char* value(const ::my_particle_filter::particle_data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_particle_filter::particle_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n\
float64 y\n\
float64 v\n\
float64 yaw_theta\n\
float64 yaw_rate\n\
";
  }

  static const char* value(const ::my_particle_filter::particle_data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_particle_filter::particle_data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.v);
      stream.next(m.yaw_theta);
      stream.next(m.yaw_rate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct particle_data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_particle_filter::particle_data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_particle_filter::particle_data_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "yaw_theta: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_theta);
    s << indent << "yaw_rate: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_rate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_PARTICLE_FILTER_MESSAGE_PARTICLE_DATA_H
