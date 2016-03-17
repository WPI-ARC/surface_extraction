//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_HPP
#define SURFACE_MANAGER_SURFACE_HPP

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <surfaces/pcl_shim/ModelCoefficients_Serialization.hpp>
#include <pcl/PolygonMesh.h>
#include <surfaces/pcl_shim/PolygonMesh_Serialization.hpp>
#include <surfaces/pcl_shim/Vertices_Serialization.hpp>
#include <surface_msgs/Surface.h>

namespace surfaces {

// Defined similarly to pcl::PointIndices
    template <typename PointType>
    struct Surface
    {
        Surface() : id(), color(), model(), concave_hull(), inliers()
        {}

        unsigned int id;
        std_msgs::ColorRGBA color;
        pcl::ModelCoefficients model;
        pcl::PolygonMesh concave_hull;
        pcl::PointCloud<PointType> inliers;

    public:
        typedef boost::shared_ptr< ::surfaces::Surface<PointType> > Ptr;
        typedef boost::shared_ptr< ::surfaces::Surface<PointType> const> ConstPtr;
    }; // struct Surface

    template <typename PointType>
    inline std::ostream& operator << (std::ostream& s, const ::surfaces::Surface<PointType> &v)
    {
        s << "id: " << std::endl;
        s << "  " << v.id;
        s << "color: " << std::endl;
        s << "  " << v.color.r << std::endl;
        s << "  " << v.color.g << std::endl;
        s << "  " << v.color.b << std::endl;
        s << "  " << v.color.a << std::endl;
        s << "model[]" << std::endl;
        for (size_t i = 0; i < v.model.values.size(); ++i)
        {
            s << "  model[" << i << "]: ";
            s << "  " << v.model.values.at(i) << std::endl;
        }
        s << "concave_hull[]" << std::endl;
        s << "  " << v.concave_hull;
        s << "inliers[]" << std::endl;
        s << "  " << v.inliers;
        return (s);
    }
}

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
                stream.next(item.model);
                stream.next(item.concave_hull);
                stream.next(item.inliers);
            }

            template<typename Stream>
            inline static void read(Stream& stream, surfaces::Surface<T>& item)
            {
                stream.next(item.id);
                stream.next(item.color);
                stream.next(item.model);
                stream.next(item.concave_hull);
                stream.next(item.inliers);
            }

            inline static int32_t serializedLength(const surfaces::Surface<T>& item)
            {
                uint32_t size = 0;
                size += serializationLength(item.id);
                size += serializationLength(item.color);
                size += serializationLength(item.model);
                size += serializationLength(item.concave_hull);
                size += serializationLength(item.inliers);
                return size;
            }

        };
    } // namespace serialization
} // namespace ros

#endif //SURFACE_MANAGER_SURFACE_HPP
