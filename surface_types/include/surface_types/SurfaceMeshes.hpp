#ifndef SURFACE_MESHES_HPP
#define SURFACE_MESHES_HPP

#include <pcl/PCLHeader.h>
#include "SurfaceMesh.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <surface_msgs2/SurfaceMeshes.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    struct SurfaceMeshes
    {
        SurfaceMeshes() : header(), latest_update(), surface_meshes()
        {}

        pcl::PCLHeader header;
        pcl::uint64_t latest_update;
        std::vector<SurfaceMesh> surface_meshes;


    public:

        operator surface_msgs2::SurfaceMeshes() const {
            surface_msgs2::SurfaceMeshes ros_surfaces;

            pcl_conversions::fromPCL(header, ros_surfaces.header);
            pcl_conversions::fromPCL(latest_update, ros_surfaces.latest_update);
            ros_surfaces.surfaces.reserve(surface_meshes.size());
            std::copy(surface_meshes.begin(), surface_meshes.end(), std::back_inserter(ros_surfaces.surfaces));

            return ros_surfaces;
        }
        typedef boost::shared_ptr< ::surface_types::SurfaceMeshes> Ptr;
        typedef boost::shared_ptr< ::surface_types::SurfaceMeshes const> ConstPtr;
    }; // struct SurfaceMeshes

    inline std::ostream& operator << (std::ostream& s, const ::surface_types::SurfaceMeshes &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
        s << "latest_update: " << std::endl;
        s << "  " << v.latest_update;
        s << "surface_meshes[]" << std::endl;
        for (size_t i = 0; i < v.surface_meshes.size(); ++i)
        {
            s << "  surface_meshes[" << i << "]: ";
            s << "  " << v.surface_meshes[i] << std::endl;
        }
        return (s);
    }
}

namespace ros
{
    namespace message_traits
    {
        template<> struct IsFixedSize<surface_types::SurfaceMeshes> : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<> struct IsSimple<surface_types::SurfaceMeshes> : public FalseType {};
        template<> struct HasHeader<surface_types::SurfaceMeshes> : public FalseType {};

        template<>
        struct MD5Sum<surface_types::SurfaceMeshes>
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs2::SurfaceMeshes>::value();
            }

            static const char* value(const surface_types::SurfaceMeshes& m)
            {
                return MD5Sum<surface_msgs2::SurfaceMeshes>::value();
            }
        };

        template<>
        struct DataType<surface_types::SurfaceMeshes>
        {
            static const char* value()
            {
                return DataType<surface_msgs2::SurfaceMeshes>::value();
            }

            static const char* value(const surface_types::SurfaceMeshes& m)
            {
                return DataType<surface_msgs2::SurfaceMeshes>::value();
            }
        };

        template<>
        struct Definition<surface_types::SurfaceMeshes>
        {
            static const char* value()
            {
                return Definition<surface_msgs2::SurfaceMeshes>::value();
            }

            static const char* value(const surface_types::SurfaceMeshes& m)
            {
                return Definition<surface_msgs2::SurfaceMeshes>::value();
            }
        };


        template<>
        struct TimeStamp<surface_types::SurfaceMeshes >
        {
            static ros::Time* pointer(surface_types::SurfaceMeshes &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surface_types::SurfaceMeshes& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surface_types::SurfaceMeshes& m) {
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
        struct Serializer<surface_types::SurfaceMeshes>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surface_types::SurfaceMeshes& item)
            {
                stream.next(item.header);
                stream.next(item.latest_update);
                stream.next(item.surface_meshes);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surface_types::SurfaceMeshes& item)
            {
                // Convert ROS header to PCL header
                std_msgs::Header ros_header;
                stream.next(ros_header);
                pcl_conversions::toPCL(ros_header, item.header);
                stream.next(item.latest_update);
                stream.next(item.surface_meshes);
            }

            inline static int32_t serializedLength(const surface_types::SurfaceMeshes& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surface_meshes);
                size += serializationLength(item.latest_update);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif // SURFACE_MESHES_HPP
