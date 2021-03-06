// Generated by gencpp from file image_transport_tutorial/ResizedImage.msg
// DO NOT EDIT!


#ifndef IMAGE_TRANSPORT_TUTORIAL_MESSAGE_RESIZEDIMAGE_H
#define IMAGE_TRANSPORT_TUTORIAL_MESSAGE_RESIZEDIMAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>

namespace image_transport_tutorial
{
template <class ContainerAllocator>
struct ResizedImage_
{
  typedef ResizedImage_<ContainerAllocator> Type;

  ResizedImage_()
    : original_height(0)
    , original_width(0)
    , image()  {
    }
  ResizedImage_(const ContainerAllocator& _alloc)
    : original_height(0)
    , original_width(0)
    , image(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _original_height_type;
  _original_height_type original_height;

   typedef uint32_t _original_width_type;
  _original_width_type original_width;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
  _image_type image;





  typedef boost::shared_ptr< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> const> ConstPtr;

}; // struct ResizedImage_

typedef ::image_transport_tutorial::ResizedImage_<std::allocator<void> > ResizedImage;

typedef boost::shared_ptr< ::image_transport_tutorial::ResizedImage > ResizedImagePtr;
typedef boost::shared_ptr< ::image_transport_tutorial::ResizedImage const> ResizedImageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::image_transport_tutorial::ResizedImage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::image_transport_tutorial::ResizedImage_<ContainerAllocator1> & lhs, const ::image_transport_tutorial::ResizedImage_<ContainerAllocator2> & rhs)
{
  return lhs.original_height == rhs.original_height &&
    lhs.original_width == rhs.original_width &&
    lhs.image == rhs.image;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::image_transport_tutorial::ResizedImage_<ContainerAllocator1> & lhs, const ::image_transport_tutorial::ResizedImage_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace image_transport_tutorial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "37d63d52feac66587bdbae1a040ffc70";
  }

  static const char* value(const ::image_transport_tutorial::ResizedImage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x37d63d52feac6658ULL;
  static const uint64_t static_value2 = 0x7bdbae1a040ffc70ULL;
};

template<class ContainerAllocator>
struct DataType< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "image_transport_tutorial/ResizedImage";
  }

  static const char* value(const ::image_transport_tutorial::ResizedImage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 original_height\n"
"uint32 original_width\n"
"sensor_msgs/Image image\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::image_transport_tutorial::ResizedImage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.original_height);
      stream.next(m.original_width);
      stream.next(m.image);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ResizedImage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::image_transport_tutorial::ResizedImage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::image_transport_tutorial::ResizedImage_<ContainerAllocator>& v)
  {
    s << indent << "original_height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.original_height);
    s << indent << "original_width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.original_width);
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMAGE_TRANSPORT_TUTORIAL_MESSAGE_RESIZEDIMAGE_H
