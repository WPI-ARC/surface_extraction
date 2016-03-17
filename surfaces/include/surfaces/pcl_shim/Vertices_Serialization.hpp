#ifndef VERTICES_SERIALIZATION_H
#define VERTICES_SERIALIZATION_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/Vertices.h>
#include <pcl_msgs/Vertices.h>

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<pcl::Vertices> : public FalseType {};
// TODO: Determine if this actually is a simple message
template<> struct IsSimple<pcl::Vertices> : public FalseType {};
template<> struct HasHeader<pcl::Vertices> : public FalseType {};

template<>
struct MD5Sum<pcl::Vertices>
{
  static const char* value()
  {
    return MD5Sum<pcl_msgs::Vertices>::value();
  }

  static const char* value(const pcl::Vertices& m)
  {
    return MD5Sum<pcl_msgs::Vertices>::value();
  }
};

template<>
struct DataType<pcl::Vertices>
{
  static const char* value()
  {
    return DataType<pcl_msgs::Vertices>::value();
  }

  static const char* value(const pcl::Vertices& m)
  {
    return DataType<pcl_msgs::Vertices>::value();
  }
};

template<>
struct Definition<pcl::Vertices>
{
  static const char* value()
  {
    return Definition<pcl_msgs::Vertices>::value();
  }

  static const char* value(const pcl::Vertices& m)
  {
    return Definition<pcl_msgs::Vertices>::value();
  }
};
} // namespace message_traits


namespace serialization
{
template<>
struct Serializer<pcl::Vertices>
{
    template<typename Stream>
    inline static void write(Stream& stream, const pcl::Vertices& item)
    {
      stream.next(item.vertices);
    }

    template<typename Stream>
    inline static void read(Stream& stream, pcl::Vertices& item)
    {
      stream.next(item.vertices);
    }

    inline static int32_t serializedLength(const pcl::Vertices& item)
    {
      uint32_t size = 0;
      size += serializationLength(item.vertices);
      return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // VERTICES_SERIALIZATION_H
