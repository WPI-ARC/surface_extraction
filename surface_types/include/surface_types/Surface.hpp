//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACE_HPP
#define SURFACE_MANAGER_SURFACE_HPP

// Contained types
#include <std_msgs/ColorRGBA.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <shape_msgs/Mesh.h>

// Manual conversion to/from ROS
#include <vector>
#include <eigen_conversions/eigen_msg.h>

// ROS message serialization
#include <surface_msgs2/Surface.h>
#include <pcl_ros/point_cloud.h>
#include <surface_types/pcl_shim/PointIndices_Serialization.hpp>
#include <surface_types/pcl_shim/ModelCoefficients_Serialization.hpp>
#include <surface_types/pcl_shim/Vertices_Serialization.hpp>

namespace surface_types {
struct Surface {
    uint32_t id;
    std_msgs::ColorRGBA color;
    pcl::PointCloud<pcl::PointXYZ> inliers;
    pcl::ModelCoefficients model;
    Eigen::Affine3d pose;
    pcl::PointIndices boundary;
    std::vector<pcl::Vertices> polygons;
    shape_msgs::Mesh mesh;

public:
    Surface() : id(0), color(), inliers(), model(), pose(), boundary(), polygons(), mesh() {}

    operator surface_msgs2::Surface() const {
        surface_msgs2::Surface s_ros;

        // ID, color, model
        s_ros.id = id;
        s_ros.color = color;
        pcl_conversions::fromPCL(model, s_ros.model);

        // Inliers
        pcl::PCLPointCloud2 inliers_pointcloud2;
        pcl::toPCLPointCloud2<pcl::PointXYZ>(inliers, inliers_pointcloud2);
        pcl_conversions::fromPCL(inliers_pointcloud2, s_ros.inliers);

        // Pose, boundary
        tf::poseEigenToMsg(pose, s_ros.pose);
        pcl_conversions::fromPCL(boundary, s_ros.boundary);

        // Vertices
        s_ros.polygons.resize(polygons.size());
        for (std::size_t i = 0; i < polygons.size(); i++) {
            pcl_conversions::fromPCL(polygons[i], s_ros.polygons[i]);
        }

        // Mesh
        s_ros.mesh = mesh;

        return s_ros;
    }

    typedef boost::shared_ptr<::surface_types::Surface> Ptr;
    typedef boost::shared_ptr<::surface_types::Surface const> ConstPtr;

    void clear_computed_values() {
        boundary = pcl::PointIndices();
        polygons.clear();
        mesh = shape_msgs::Mesh();
    }
};

// struct Surface

inline std::ostream &operator<<(std::ostream &s, const Surface &v) {
    s << "id: " << std::endl;
    s << "  " << v.id;
    s << "color: " << std::endl;
    s << "  " << v.color.r << ", " << v.color.g << ", " << v.color.b << ", " << v.color.a << std::endl;
    s << "model:" << std::endl;
    s << "  " << v.model.values[0] << ", " << v.model.values[1];
    s << ", " << v.model.values[2] << ", " << v.model.values[3] << std::endl;
    s << "inliers[" << v.inliers.size() << "]:" << std::endl;
    for (auto &inlier : v.inliers)
        s << "  " << inlier << std::endl;
    s << "pose:" << std::endl;
    // Print eigen transform as a matrix using .format() to set the indentation
    s << v.pose.matrix().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, " ", "\n", "  ")) << std::endl;
    s << "boundary[" << v.boundary.indices.size() << "]:" << std::endl;
    for (auto &b : v.boundary.indices)
        s << "  " << b << std::endl;
    s << "polygons[" << v.polygons.size() << "]:" << std::endl;
    for (auto &vertices : v.polygons) {
        s << "  vertices[" << vertices.vertices.size() << "]:" << std::endl;
        for (auto &vertex : vertices.vertices)
            s << "    " << vertex << std::endl;
    }
    s << "mesh:" << std::endl;
    s << "  " << v.mesh << std::endl; // You're on your own for this one
    return (s);
}
}

namespace ros {
namespace message_traits {
template <>
struct IsFixedSize<surface_types::Surface> : public FalseType {};
template <>
struct IsSimple<surface_types::Surface> : public FalseType {};
template <>
struct HasHeader<surface_types::Surface> : public FalseType {};

template <>
struct MD5Sum<surface_types::Surface> {
    static const char *value() { return MD5Sum<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::Surface &m) { return MD5Sum<surface_msgs2::Surface>::value(); }
};

template <>
struct DataType<surface_types::Surface> {
    static const char *value() { return DataType<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::Surface &m) { return DataType<surface_msgs2::Surface>::value(); }
};

template <>
struct Definition<surface_types::Surface> {
    static const char *value() { return Definition<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::Surface &m) { return Definition<surface_msgs2::Surface>::value(); }
};
} // namespace message_traits

namespace serialization {
template <>
struct Serializer<surface_types::Surface> {
    template <typename Stream>
    inline static void write(Stream &stream, const surface_types::Surface &s) {
        geometry_msgs::Pose geom_pose;
        tf::poseEigenToMsg(s.pose, geom_pose);

        stream.next(s.id);
        stream.next(s.color);
        stream.next(s.model);
        stream.next(s.inliers);
        stream.next(geom_pose);
        stream.next(s.boundary);
        stream.next(s.polygons);
        stream.next(s.mesh);
    }

    template <typename Stream>
    inline static void read(Stream &stream, surface_types::Surface &s) {
        geometry_msgs::Pose geom_pose;

        stream.next(s.id);
        stream.next(s.color);
        stream.next(s.model);
        stream.next(s.inliers);
        stream.next(geom_pose);
        stream.next(s.boundary);
        stream.next(s.polygons);
        stream.next(s.mesh);

        tf::poseMsgToEigen(geom_pose, s.pose);
    }

    inline static uint32_t serializedLength(const surface_types::Surface &s) {
        geometry_msgs::Pose geom_pose;
        tf::poseEigenToMsg(s.pose, geom_pose);

        uint32_t size = 0;

        size += serializationLength(s.id);
        size += serializationLength(s.color);
        size += serializationLength(s.model);
        size += serializationLength(s.inliers);
        size += serializationLength(geom_pose);
        size += serializationLength(s.boundary);
        size += serializationLength(s.polygons);
        size += serializationLength(s.mesh);

        return size;
    }

};
} // namespace serialization
} // namespace ros

#endif // SURFACE_MANAGER_SURFACE_HPP
