#ifndef MODELCOEFFICIENTS_SERIALIZATION_H
#define MODELCOEFFICIENTS_SERIALIZATION_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_msgs/ModelCoefficients.h>

namespace ros
{
namespace message_traits
{
template<> struct IsFixedSize<pcl::ModelCoefficients> : public FalseType {};
// TODO: Determine if this actually is a simple message
template<> struct IsSimple<pcl::ModelCoefficients> : public FalseType {};
template<> struct HasHeader<pcl::ModelCoefficients> : public TrueType {};

template<>
struct MD5Sum<pcl::ModelCoefficients>
{
  static const char* value()
  {
    return MD5Sum<pcl_msgs::ModelCoefficients>::value();
  }

  static const char* value(const pcl::ModelCoefficients& m)
  {
    return MD5Sum<pcl_msgs::ModelCoefficients>::value();
  }
};

template<>
struct DataType<pcl::ModelCoefficients>
{
  static const char* value()
  {
    return DataType<pcl_msgs::ModelCoefficients>::value();
  }

  static const char* value(const pcl::ModelCoefficients& m)
  {
    return DataType<pcl_msgs::ModelCoefficients>::value();
  }
};

template<>
struct Definition<pcl::ModelCoefficients>
{
  static const char* value()
  {
    return Definition<pcl_msgs::ModelCoefficients>::value();
  }

  static const char* value(const pcl::ModelCoefficients& m)
  {
    return Definition<pcl_msgs::ModelCoefficients>::value();
  }
};


template<>
struct TimeStamp<pcl::ModelCoefficients>
{
  static ros::Time* pointer(pcl::ModelCoefficients &m) {
    header_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_));
    return &(header_->stamp);
  }
  static ros::Time const* pointer(const pcl::ModelCoefficients& m) {
    header_const_.reset(new std_msgs::Header());
    pcl_conversions::fromPCL(m.header, *(header_const_));
    return &(header_const_->stamp);
  }
  static ros::Time value(const pcl::ModelCoefficients& m) {
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
struct Serializer<pcl::ModelCoefficients>
{
    template<typename Stream>
    inline static void write(Stream& stream, const pcl::ModelCoefficients& item)
    {
      stream.next(item.header);
      stream.next(item.values);
    }

    template<typename Stream>
    inline static void read(Stream& stream, pcl::ModelCoefficients& item)
    {
      // Convert ROS header to PCL header
      std_msgs::Header ros_header;
      stream.next(ros_header);
      pcl_conversions::toPCL(ros_header, item.header);
      stream.next(item.values);
    }

    inline static int32_t serializedLength(const pcl::ModelCoefficients& item)
    {
      uint32_t size = 0;
      size += serializationLength(item.header);
      size += serializationLength(item.values);
      return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // MODELCOEFFICIENTS_SERIALIZATION_H
