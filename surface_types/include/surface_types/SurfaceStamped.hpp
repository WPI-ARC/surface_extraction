//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_STAMPED_HPP
#define SURFACE_MANAGER_SURFACE_STAMPED_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include "Surface.hpp"
#include <surface_msgs2/SurfaceStamped.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    struct SurfaceStamped
    {
        SurfaceStamped() : header(), surface()
        {}

        ::pcl::PCLHeader header;
        ::surface_types::Surface surface;


    public:
        typedef boost::shared_ptr< ::surface_types::SurfaceStamped> Ptr;
        typedef boost::shared_ptr< ::surface_types::SurfaceStamped const> ConstPtr;
    }; // struct SurfaceStamped

    inline std::ostream& operator << (std::ostream& s, const ::surface_types::SurfaceStamped &v)
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
        struct IsFixedSize<surface_types::SurfaceStamped> : public FalseType {};
        struct IsSimple<surface_types::SurfaceStamped> : public FalseType {};
        struct HasHeader<surface_types::SurfaceStamped> : public FalseType {};

        struct MD5Sum<surface_types::SurfaceStamped>
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfaceStamped& m)
            {
                return MD5Sum<surface_msgs2::SurfacesStamped>::value();
            }
        };

        struct DataType<surface_types::SurfaceStamped>
        {
            static const char* value()
            {
                return DataType<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfaceStamped& m)
            {
                return DataType<surface_msgs2::SurfacesStamped>::value();
            }
        };

        struct Definition<surface_types::SurfaceStamped>
        {
            static const char* value()
            {
                return Definition<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfaceStamped& m)
            {
                return Definition<surface_msgs2::SurfacesStamped>::value();
            }
        };

        struct TimeStamp<surface_types::SurfaceStamped>
        {
            static ros::Time* pointer(surface_types::SurfaceStamped &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surface_types::SurfaceStamped& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surface_types::SurfaceStamped& m) {
                return pcl_conversions::fromPCL(m.header).stamp;
            }
        private:
            static boost::shared_ptr<std_msgs::Header> header_;
            static boost::shared_ptr<std_msgs::Header> header_const_;
        };

    } // namespace message_traits

    namespace serialization
    {
        struct Serializer<surface_types::SurfaceStamped>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surface_types::SurfaceStamped& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surface_types::SurfaceStamped& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            inline static int32_t serializedLength(const surface_types::SurfaceStamped& item)
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
