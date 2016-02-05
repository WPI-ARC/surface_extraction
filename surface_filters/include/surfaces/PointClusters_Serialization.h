#ifndef POINT_CLUSTERS_SERIALIZATION_H
#define POINT_CLUSTERS_SERIALIZATION_H

#include <pcl_conversions/pcl_conversions.h>
#include <surfaces/PointClusters.h>
#include <surface_msgs/PointClusters.h>
#include <surfaces/PointIndices_Serialization.h>
#include <surfaces/ModelCoefficients_Serialization.h>

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<surfaces::PointClusters> : public FalseType {};
// TODO: Determine if this actually is a simple message
template<> struct IsSimple<surfaces::PointClusters> : public FalseType {};
template<> struct HasHeader<surfaces::PointClusters> : public TrueType {};

template<>
struct MD5Sum<surfaces::PointClusters>
{
  static const char* value()
  {
    return MD5Sum<surface_msgs::PointClusters>::value();
  }

  static const char* value(const surfaces::PointClusters& m)
  {
    return MD5Sum<surface_msgs::PointClusters>::value();
  }
};

template<>
struct DataType<surfaces::PointClusters>
{
  static const char* value()
  {
    return DataType<surface_msgs::PointClusters>::value();
  }

  static const char* value(const surfaces::PointClusters& m)
  {
    return DataType<surface_msgs::PointClusters>::value();
  }
};

template<>
struct Definition<surfaces::PointClusters>
{
  static const char* value()
  {
    return Definition<surface_msgs::PointClusters>::value();
  }

  static const char* value(const surfaces::PointClusters& m)
  {
    return Definition<surface_msgs::PointClusters>::value();
  }
};


template<>
struct TimeStamp<surfaces::PointClusters>
{
  static ros::Time* pointer(surfaces::PointClusters &m) {
    header_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_));
    return &(header_->stamp);
  }
  static ros::Time const* pointer(const surfaces::PointClusters& m) {
    header_const_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_const_));
    return &(header_const_->stamp);
  }
  static ros::Time value(const surfaces::PointClusters& m) {
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
struct Serializer<surfaces::PointClusters>
{
    template<typename Stream>
    inline static void write(Stream& stream, const surfaces::PointClusters& item)
    {
      stream.next(item.header);
      stream.next(item.has_models);
      stream.next(item.clusters);
      stream.next(item.models);
    }

    template<typename Stream>
    inline static void read(Stream& stream, surfaces::PointClusters& item)
    {
      // Convert ROS header to PCL header
      std_msgs::Header ros_header;
      stream.next(ros_header);
      pcl_conversions::toPCL(ros_header, item.header);
      stream.next(item.has_models);
      stream.next(item.clusters);
      stream.next(item.models);
    }

    inline static int32_t serializedLength(const surfaces::PointClusters& item)
    {
      uint32_t size = 0;
      size += serializationLength(item.header);
      size += serializationLength(item.has_models);
      size += serializationLength(item.clusters);
      size += serializationLength(item.models);
      return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // POINT_CLUSTERS_SERIALIZATION_H
