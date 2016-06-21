//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_STAMPED_HPP
#define SURFACE_MANAGER_SURFACE_STAMPED_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include "Surface.hpp"
#include <surface_msgs2/SurfacesStamped.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct SurfacesStamped
    {
        SurfacesStamped() : header(), surface()
        {}

        ::pcl::PCLHeader header;
        ::surface_types::Surface<PointType> surface;


    public:
        typedef boost::shared_ptr< ::surface_types::SurfacesStamped<PointType> > Ptr;
        typedef boost::shared_ptr< ::surface_types::SurfacesStamped<PointType> const> ConstPtr;
    }; // struct SurfacesStamped

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surface_types::SurfacesStamped<PointType> &v)
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
        template<typename T> struct IsFixedSize<surface_types::SurfacesStamped<T> > : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<typename T> struct IsSimple<surface_types::SurfacesStamped<T> > : public FalseType {};
        template<typename T> struct HasHeader<surface_types::SurfacesStamped<T> > : public FalseType {};

        template<typename T>
        struct MD5Sum<surface_types::SurfacesStamped<T> >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfacesStamped<T>& m)
            {
                return MD5Sum<surface_msgs2::SurfacesStamped>::value();
            }
        };

        template<typename T>
        struct DataType<surface_types::SurfacesStamped<T> >
        {
            static const char* value()
            {
                return DataType<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfacesStamped<T>& m)
            {
                return DataType<surface_msgs2::SurfacesStamped>::value();
            }
        };

        template<typename T>
        struct Definition<surface_types::SurfacesStamped<T> >
        {
            static const char* value()
            {
                return Definition<surface_msgs2::SurfacesStamped>::value();
            }

            static const char* value(const surface_types::SurfacesStamped<T>& m)
            {
                return Definition<surface_msgs2::SurfacesStamped>::value();
            }
        };

        template<typename T>
        struct TimeStamp<surface_types::SurfacesStamped<T> >
        {
            static ros::Time* pointer(surface_types::SurfacesStamped<T> &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surface_types::SurfacesStamped<T>& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surface_types::SurfacesStamped<T>& m) {
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
        struct Serializer<surface_types::SurfacesStamped<T> >
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surface_types::SurfacesStamped<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surface_types::SurfacesStamped<T>& item)
            {
                stream.next(item.header);
                stream.next(item.surface);
            }

            inline static int32_t serializedLength(const surface_types::SurfacesStamped<T>& item)
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
