#ifndef POINT_INDICES_SERIALIZATION_H
#define POINT_INDICES_SERIALIZATION_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PointIndices.h>
#include <pcl_msgs/PointIndices.h>

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<pcl::PointIndices> : public FalseType {};
// TODO: Determine if this actually is a simple message
template<> struct IsSimple<pcl::PointIndices> : public FalseType {};
template<> struct HasHeader<pcl::PointIndices> : public TrueType {};

template<>
struct MD5Sum<pcl::PointIndices>
{
  static const char* value()
  {
    return MD5Sum<pcl_msgs::PointIndices>::value();
  }

  static const char* value(const pcl::PointIndices& m)
  {
    return MD5Sum<pcl_msgs::PointIndices>::value();
  }
};

template<>
struct DataType<pcl::PointIndices>
{
  static const char* value()
  {
    return DataType<pcl_msgs::PointIndices>::value();
  }

  static const char* value(const pcl::PointIndices& m)
  {
    return DataType<pcl_msgs::PointIndices>::value();
  }
};

template<>
struct Definition<pcl::PointIndices>
{
  static const char* value()
  {
    return Definition<pcl_msgs::PointIndices>::value();
  }

  static const char* value(const pcl::PointIndices& m)
  {
    return Definition<pcl_msgs::PointIndices>::value();
  }
};


template<>
struct TimeStamp<pcl::PointIndices>
{
  static ros::Time* pointer(pcl::PointIndices &m) {
    header_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_));
    return &(header_->stamp);
  }
  static ros::Time const* pointer(const pcl::PointIndices& m) {
    header_const_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_const_));
    return &(header_const_->stamp);
  }
  static ros::Time value(const pcl::PointIndices& m) {
    return pcl_conversions::fromPCL(m.header).stamp;
  }
private:
  static boost::shared_ptr<std_msgs::Header> header_;
  static boost::shared_ptr<std_msgs::Header> header_const_;
};
} // namespace message_traits

namespace serialization
{
template<>
struct Serializer<pcl::PointIndices>
{
    template<typename Stream>
    inline static void write(Stream& stream, const pcl::PointIndices& item)
    {
      stream.next(item.header);
      stream.next(item.indices);
    }

    template<typename Stream>
    inline static void read(Stream& stream, pcl::PointIndices& item)
    {
      // Convert ROS header to PCL header
      std_msgs::Header ros_header;
      stream.next(ros_header);
      pcl_conversions::toPCL(ros_header, item.header);
      stream.next(item.indices);
    }

    inline static int32_t serializedLength(const pcl::PointIndices& item)
    {
      uint32_t size = 0;
      size += serializationLength(item.header);
      size += serializationLength(item.indices);
      return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // POINT_INDICES_SERIALIZATION_H
