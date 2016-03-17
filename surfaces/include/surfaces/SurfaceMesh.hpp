//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_MESH_HPP
#define SURFACE_MANAGER_SURFACE_MESH_HPP

#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <surface_msgs/SurfaceMesh.h>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    struct SurfaceMesh
    {
        SurfaceMesh() : id(), surface_mesh()
        {}

        unsigned int id;
        shape_msgs::Mesh surface_mesh;

    public:
        typedef boost::shared_ptr< ::surfaces::SurfaceMesh> Ptr;
        typedef boost::shared_ptr< ::surfaces::SurfaceMesh const> ConstPtr;
    }; // struct Surface

    inline std::ostream& operator << (std::ostream& s, const ::surfaces::SurfaceMesh &v)
    {
        s << "id: " << std::endl;
        s << "  " << v.id;
        s << "surface_mesh: " << std::endl;
        s << "  " << v.surface_mesh;
        return (s);
    }
}

namespace ros {
    namespace message_traits {
        template<>
        struct IsFixedSize<surfaces::SurfaceMesh> : public FalseType {
        };
// TODO: Determine if this actually is a simple message
        template<>
        struct IsSimple<surfaces::SurfaceMesh> : public FalseType {
        };
        template<>
        struct HasHeader<surfaces::SurfaceMesh> : public FalseType {
        };

        template<>
        struct MD5Sum<surfaces::SurfaceMesh> {
            static const char *value() {
                return MD5Sum<surface_msgs::SurfaceMesh>::value();
            }

            static const char *value(const surfaces::SurfaceMesh &m) {
                return MD5Sum<surface_msgs::SurfaceMesh>::value();
            }
        };

        template<>
        struct DataType<surfaces::SurfaceMesh> {
            static const char *value() {
                return DataType<surface_msgs::SurfaceMesh>::value();
            }

            static const char *value(const surfaces::SurfaceMesh &m) {
                return DataType<surface_msgs::SurfaceMesh>::value();
            }
        };

        template<>
        struct Definition<surfaces::SurfaceMesh> {
            static const char *value() {
                return Definition<surface_msgs::SurfaceMesh>::value();
            }

            static const char *value(const surfaces::SurfaceMesh &m) {
                return Definition<surface_msgs::SurfaceMesh>::value();
            }
        };

    } // namespace message_traits

    namespace serialization {
        template<>
        struct Serializer<surfaces::SurfaceMesh> {
            template<typename Stream>
            inline static void write(Stream &stream, const surfaces::SurfaceMesh &item) {
                stream.next(item.id);
                stream.next(item.surface_mesh);
            }

            template<typename Stream>
            inline static void read(Stream &stream, surfaces::SurfaceMesh &item) {
                stream.next(item.id);
                stream.next(item.surface_mesh);
            }

            inline static int32_t serializedLength(const surfaces::SurfaceMesh &item) {
                uint32_t size = 0;
                size += serializationLength(item.id);
                size += serializationLength(item.surface_mesh);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif //SURFACE_MANAGER_SURFACE_MESH_HPP
