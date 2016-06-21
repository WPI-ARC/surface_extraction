//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_MESH_HPP
#define SURFACE_MANAGER_SURFACE_MESH_HPP

#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <surface_msgs2/SurfaceMesh.h>

namespace surface_types {

// Defined similarly to pcl::PointIndices
    struct SurfaceMesh
    {
        SurfaceMesh() : id(), surface_mesh()
        {}

        unsigned int id;
        shape_msgs::Mesh surface_mesh;

    public:
        operator surface_msgs2::SurfaceMesh () const {
            surface_msgs2::SurfaceMesh m_ros;

            m_ros.id = id;
            m_ros.mesh = surface_mesh;

            return m_ros;
        }


        typedef boost::shared_ptr< ::surface_types::SurfaceMesh> Ptr;
        typedef boost::shared_ptr< ::surface_types::SurfaceMesh const> ConstPtr;
    }; // struct Surface

    inline std::ostream& operator << (std::ostream& s, const ::surface_types::SurfaceMesh &v)
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
        struct IsFixedSize<surface_types::SurfaceMesh> : public FalseType {
        };
// TODO: Determine if this actually is a simple message
        template<>
        struct IsSimple<surface_types::SurfaceMesh> : public FalseType {
        };
        template<>
        struct HasHeader<surface_types::SurfaceMesh> : public FalseType {
        };

        template<>
        struct MD5Sum<surface_types::SurfaceMesh> {
            static const char *value() {
                return MD5Sum<surface_msgs2::SurfaceMesh>::value();
            }

            static const char *value(const surface_types::SurfaceMesh &m) {
                return MD5Sum<surface_msgs2::SurfaceMesh>::value();
            }
        };

        template<>
        struct DataType<surface_types::SurfaceMesh> {
            static const char *value() {
                return DataType<surface_msgs2::SurfaceMesh>::value();
            }

            static const char *value(const surface_types::SurfaceMesh &m) {
                return DataType<surface_msgs2::SurfaceMesh>::value();
            }
        };

        template<>
        struct Definition<surface_types::SurfaceMesh> {
            static const char *value() {
                return Definition<surface_msgs2::SurfaceMesh>::value();
            }

            static const char *value(const surface_types::SurfaceMesh &m) {
                return Definition<surface_msgs2::SurfaceMesh>::value();
            }
        };

    } // namespace message_traits

    namespace serialization {
        template<>
        struct Serializer<surface_types::SurfaceMesh> {
            template<typename Stream>
            inline static void write(Stream &stream, const surface_types::SurfaceMesh &item) {
                stream.next(item.id);
                stream.next(item.surface_mesh);
            }

            template<typename Stream>
            inline static void read(Stream &stream, surface_types::SurfaceMesh &item) {
                stream.next(item.id);
                stream.next(item.surface_mesh);
            }

            inline static int32_t serializedLength(const surface_types::SurfaceMesh &item) {
                uint32_t size = 0;
                size += serializationLength(item.id);
                size += serializationLength(item.surface_mesh);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif //SURFACE_MANAGER_SURFACE_MESH_HPP
