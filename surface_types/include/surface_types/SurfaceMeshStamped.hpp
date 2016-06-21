//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_MESH_STAMPED_HPP
#define SURFACE_MANAGER_SURFACE_MESH_STAMPED_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include "SurfaceMesh.hpp"
#include <surface_msgs2/SurfaceMeshStamped.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    struct SurfaceMeshStamped
    {
        SurfaceMeshStamped() : header(), surface_mesh()
        {}

        ::pcl::PCLHeader header;
        ::surface_types::SurfaceMesh surface_mesh;


    public:
        typedef boost::shared_ptr< ::surface_types::SurfaceMeshStamped > Ptr;
        typedef boost::shared_ptr< ::surface_types::SurfaceMeshStamped const> ConstPtr;
    }; // struct SurfaceMeshStamped

    inline std::ostream& operator << (std::ostream& s, const ::surface_types::SurfaceMeshStamped &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "surface_mesh: " << std::endl;
        s << "  " << v.surface_mesh;
        return (s);
    }
}

namespace ros
{
    namespace message_traits
    {
        template<> struct IsFixedSize<surface_types::SurfaceMeshStamped > : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<> struct IsSimple<surface_types::SurfaceMeshStamped > : public FalseType {};
        template<> struct HasHeader<surface_types::SurfaceMeshStamped > : public FalseType {};

        template<> struct MD5Sum<surface_types::SurfaceMeshStamped >
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs2::SurfaceMeshStamped>::value();
            }

            static const char* value(const surface_types::SurfaceMeshStamped& m)
            {
                return MD5Sum<surface_msgs2::SurfaceMeshStamped>::value();
            }
        };

        template<> struct DataType<surface_types::SurfaceMeshStamped>
        {
            static const char* value()
            {
                return DataType<surface_msgs2::SurfaceMeshStamped>::value();
            }

            static const char* value(const surface_types::SurfaceMeshStamped& m)
            {
                return DataType<surface_msgs2::SurfaceMeshStamped>::value();
            }
        };

        template<> struct Definition<surface_types::SurfaceMeshStamped>
        {
            static const char* value()
            {
                return Definition<surface_msgs2::SurfaceMeshStamped>::value();
            }

            static const char* value(const surface_types::SurfaceMeshStamped& m)
            {
                return Definition<surface_msgs2::SurfaceMeshStamped>::value();
            }
        };

        template<> struct TimeStamp<surface_types::SurfaceMeshStamped>
        {
            static ros::Time* pointer(surface_types::SurfaceMeshStamped &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surface_types::SurfaceMeshStamped& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surface_types::SurfaceMeshStamped& m) {
                return pcl_conversions::fromPCL(m.header).stamp;
            }
        private:
            static boost::shared_ptr<std_msgs::Header> header_;
            static boost::shared_ptr<std_msgs::Header> header_const_;
        };

    } // namespace message_traits

    namespace serialization
    {
        template<> struct Serializer<surface_types::SurfaceMeshStamped>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surface_types::SurfaceMeshStamped& item)
            {
                stream.next(item.header);
                stream.next(item.surface_mesh);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surface_types::SurfaceMeshStamped& item)
            {
                stream.next(item.header);
                stream.next(item.surface_mesh);
            }

            inline static int32_t serializedLength(const surface_types::SurfaceMeshStamped& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surface_mesh);
                return size;
            }

        };
    } // namespace serialization

} // namespace ros

#endif //SURFACE_MANAGER_SURFACE_MESH_STAMPED_HPP
