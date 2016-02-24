#ifndef SURFACE_SERIALIZATION_HPP
#define SURFACE_SERIALIZATION_HPP

#include <pcl_conversions/pcl_conversions.h>
#include <surfaces/Surface.hpp>
#include <surface_msgs/Surface.h>

namespace ros
{
namespace message_traits
{
template<typename T> struct IsFixedSize<surfaces::Surface<T> > : public FalseType {};
// TODO: Determine if this actually is a simple message
template<typename T> struct IsSimple<surfaces::Surface<T> > : public FalseType {};
template<typename T> struct HasHeader<surfaces::Surface<T> > : public FalseType {};

template<typename T>
struct MD5Sum<surfaces::Surface<T> >
{
  static const char* value()
  {
    return MD5Sum<surface_msgs::Surface>::value();
  }

  static const char* value(const surfaces::Surface<T>& m)
  {
    return MD5Sum<surface_msgs::Surface>::value();
  }
};

template<typename T>
struct DataType<surfaces::Surface<T> >
{
  static const char* value()
  {
    return DataType<surface_msgs::Surface>::value();
  }

  static const char* value(const surfaces::Surface<T>& m)
  {
    return DataType<surface_msgs::Surface>::value();
  }
};

template<typename T>
struct Definition<surfaces::Surface<T> >
{
  static const char* value()
  {
    return Definition<surface_msgs::Surface>::value();
  }

  static const char* value(const surfaces::Surface<T>& m)
  {
    return Definition<surface_msgs::Surface>::value();
  }
};
} // namespace message_traits

namespace serialization
{
template<typename T>
struct Serializer<surfaces::Surface<T> >
{
    template<typename Stream>
    inline static void write(Stream& stream, const surfaces::Surface<T>& item)
    {
        stream.next(item.id);
        stream.next(item.color);
        stream.next(item.inliers);
        stream.next(item.convex_hull);
        stream.next(item.plane);
    }

    template<typename Stream>
    inline static void read(Stream& stream, surfaces::Surface<T>& item)
    {
        stream.next(item.id);
        stream.next(item.color);
        stream.next(item.inliers);
        stream.next(item.convex_hull);
        stream.next(item.plane);
    }

    inline static int32_t serializedLength(const surfaces::Surface<T>& item)
    {
        uint32_t size = 0;
        size += serializationLength(item.id);
        size += serializationLength(item.color);
        size += serializationLength(item.inliers);
        size += serializationLength(item.convex_hull);
        size += serializationLength(item.plane);
        return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // SURFACE_SERIALIZATION_HPP
