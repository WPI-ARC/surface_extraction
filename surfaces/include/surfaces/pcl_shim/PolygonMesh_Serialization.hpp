#ifndef POLYGONMESH_SERIALIZATION_H
#define POLYGONMESH_SERIALIZATION_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<pcl::PolygonMesh> : public FalseType {};
// TODO: Determine if this actually is a simple message
template<> struct IsSimple<pcl::PolygonMesh> : public FalseType {};
template<> struct HasHeader<pcl::PolygonMesh> : public TrueType {};

template<>
struct MD5Sum<pcl::PolygonMesh>
{
  static const char* value()
  {
    return MD5Sum<pcl_msgs::PolygonMesh>::value();
  }

  static const char* value(const pcl::PolygonMesh& m)
  {
    return MD5Sum<pcl_msgs::PolygonMesh>::value();
  }
};

template<>
struct DataType<pcl::PolygonMesh>
{
  static const char* value()
  {
    return DataType<pcl_msgs::PolygonMesh>::value();
  }

  static const char* value(const pcl::PolygonMesh& m)
  {
    return DataType<pcl_msgs::PolygonMesh>::value();
  }
};

template<>
struct Definition<pcl::PolygonMesh>
{
  static const char* value()
  {
    return Definition<pcl_msgs::PolygonMesh>::value();
  }

  static const char* value(const pcl::PolygonMesh& m)
  {
    return Definition<pcl_msgs::PolygonMesh>::value();
  }
};


template<>
struct TimeStamp<pcl::PolygonMesh>
{
  static ros::Time* pointer(pcl::PolygonMesh &m) {
    header_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_));
    return &(header_->stamp);
  }
  static ros::Time const* pointer(const pcl::PolygonMesh& m) {
    header_const_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_const_));
    return &(header_const_->stamp);
  }
  static ros::Time value(const pcl::PolygonMesh& m) {
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
struct Serializer<pcl::PolygonMesh>
{
    template<typename Stream>
    inline static void write(Stream& stream, const pcl::PolygonMesh& item)
    {
      stream.next(item.header);
        stream.next(item.cloud);
        stream.next(item.polygons);
    }

    template<typename Stream>
    inline static void read(Stream& stream, pcl::PolygonMesh& item)
    {
      // Convert ROS header to PCL header
      std_msgs::Header ros_header;
      stream.next(ros_header);
      pcl_conversions::toPCL(ros_header, item.header);
        stream.next(item.cloud);
        stream.next(item.polygons);
    }

    inline static int32_t serializedLength(const pcl::PolygonMesh& item)
    {
      uint32_t size = 0;
      size += serializationLength(item.header);
        size += serializationLength(item.cloud);
        size += serializationLength(item.polygons);
      return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // POLYGONMESH_SERIALIZATION_H
