//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_STAMPED_HPP
#define SURFACE_MANAGER_SURFACE_STAMPED_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include "Surface.hpp"
#include <surface_msgs/SurfaceStamped.h>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct SurfaceStamped
    {
        SurfaceStamped() : header(), surface()
        {}

        ::pcl::PCLHeader header;
        ::surfaces::Surface<PointType> surface;


    public:
        typedef boost::shared_ptr< ::surfaces::SurfaceStamped<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::SurfaceStamped<PointType> const> ConstPtr;
    }; // struct SurfaceStamped

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::SurfaceStamped<PointType> &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "surface: " << std::endl;
        s << "  " << v.surface;
        return (s);
    }
}

namespace ros
{
    namespace message_traits
    {
        template<typename T> struct IsFixedSize<surfaces::SurfaceStamped<T> > : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<typename T> struct IsSimple<surfaces::SurfaceStamped<T> > : public FalseType {};
        template<typename T> struct HasHeader<surfaces::SurfaceStamped<T> > : public FalseType {};

        template<typename T>
        struct MD5Sum<surfaces::SurfaceStamped<T> >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs::SurfaceStamped>::value();
            }

            static const char* value(const surfaces::SurfaceStamped<T>& m)
            {
                return MD5Sum<surface_msgs::SurfaceStamped>::value();
            }
        };

        template<typename T>
        struct DataType<surfaces::SurfaceStamped<T> >
        {
            static const char* value()
            {
                return DataType<surface_msgs::SurfaceStamped>::value();
            }

            static const char* value(const surfaces::SurfaceStamped<T>& m)
            {
                return DataType<surface_msgs::SurfaceStamped>::value();
            }
        };

        template<typename T>
        struct Definition<surfaces::SurfaceStamped<T> >
        {
            static const char* value()
            {
                return Definition<surface_msgs::SurfaceStamped>::value();
            }

            static const char* value(const surfaces::SurfaceStamped<T>& m)
            {
                return Definition<surface_msgs::SurfaceStamped>::value();
            }
        };

        template<typename T>
        struct TimeStamp<surfaces::SurfaceStamped<T> >
        {
            static ros::Time* pointer(surfaces::SurfaceStamped<T> &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surfaces::SurfaceStamped<T>& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surfaces::SurfaceStamped<T>& m) {
                return pcl_conversions::fromPCL(m.header).stamp;
            }
        private:
            static boost::shared_ptr<std_msgs::Header> header_;
            static boost::shared_ptr<std_msgs::Header> header_const_;
        };

    } // namespace message_traits

    namespace serialization
    {
        template<typename T>
        struct Serializer<surfaces::SurfaceStamped<T> >
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surfaces::SurfaceStamped<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::SurfaceStamped<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            inline static int32_t serializedLength(const surfaces::SurfaceStamped<T>& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surface);
                return size;
            }

        };
    } // namespace serialization

} // namespace ros

#endif //SURFACE_MANAGER_SURFACE_STAMPED_HPP
