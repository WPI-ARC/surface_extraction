#ifndef SURFACE_MESHES_HPP
#define SURFACE_MESHES_HPP

#include <pcl/PCLHeader.h>
#include "SurfaceMesh.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <surface_msgs/SurfaceMeshes.h>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    struct SurfaceMeshes
    {
        SurfaceMeshes() : header(), surface_meshes()
        {}

        pcl::PCLHeader header;
        std::vector<SurfaceMesh> surface_meshes;


    public:
        typedef boost::shared_ptr< ::surfaces::SurfaceMeshes> Ptr;
        typedef boost::shared_ptr< ::surfaces::SurfaceMeshes const> ConstPtr;
    }; // struct SurfaceMeshes

    inline std::ostream& operator << (std::ostream& s, const ::surfaces::SurfaceMeshes &v)
    {
        s << "header: " << std::endl;
        s << "  " << v.header;
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
        template<> struct IsFixedSize<surfaces::SurfaceMeshes> : public FalseType {};
// TODO: Determine if this actually is a simple message
        template<> struct IsSimple<surfaces::SurfaceMeshes> : public FalseType {};
        template<> struct HasHeader<surfaces::SurfaceMeshes> : public FalseType {};

        template<>
        struct MD5Sum<surfaces::SurfaceMeshes>
        {
            static const char* value()
            {
                return MD5Sum<surface_msgs::SurfaceMeshes>::value();
            }

            static const char* value(const surfaces::SurfaceMeshes& m)
            {
                return MD5Sum<surface_msgs::SurfaceMeshes>::value();
            }
        };

        template<>
        struct DataType<surfaces::SurfaceMeshes>
        {
            static const char* value()
            {
                return DataType<surface_msgs::SurfaceMeshes>::value();
            }

            static const char* value(const surfaces::SurfaceMeshes& m)
            {
                return DataType<surface_msgs::SurfaceMeshes>::value();
            }
        };

        template<>
        struct Definition<surfaces::SurfaceMeshes>
        {
            static const char* value()
            {
                return Definition<surface_msgs::SurfaceMeshes>::value();
            }

            static const char* value(const surfaces::SurfaceMeshes& m)
            {
                return Definition<surface_msgs::SurfaceMeshes>::value();
            }
        };


        template<>
        struct TimeStamp<surfaces::SurfaceMeshes >
        {
            static ros::Time* pointer(surfaces::SurfaceMeshes &m) {
                header_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_));
                return &(header_->stamp);
            }
            static ros::Time const* pointer(const surfaces::SurfaceMeshes& m) {
                header_const_.reset(new std_msgs::Header());
                pcl_conversions::fromPCL(m.header, *(header_const_));
                return &(header_const_->stamp);
            }
            static ros::Time value(const surfaces::SurfaceMeshes& m) {
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
        struct Serializer<surfaces::SurfaceMeshes>
        {
            template<typename Stream>
            inline static void write(Stream& stream, const surfaces::SurfaceMeshes& item)
            {
                stream.next(item.header);
                stream.next(item.surface_meshes);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::SurfaceMeshes& item)
            {
                // Convert ROS header to PCL header
                std_msgs::Header ros_header;
                stream.next(ros_header);
                pcl_conversions::toPCL(ros_header, item.header);
                stream.next(item.surface_meshes);
            }

            inline static int32_t serializedLength(const surfaces::SurfaceMeshes& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.header);
                size += serializationLength(item.surface_meshes);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif // SURFACE_MESHES_HPP
