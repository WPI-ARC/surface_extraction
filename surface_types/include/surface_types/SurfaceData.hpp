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
#include <surface_msgs2/Surface.h>
#include <vector>
#include <eigen_conversions/eigen_msg.h>

// ROS message serialization
#include <pcl_ros/point_cloud.h>
#include <surface_types/pcl_shim/PointIndices_Serialization.hpp>
#include <surface_types/pcl_shim/ModelCoefficients_Serialization.hpp>
#include <surface_types/pcl_shim/Vertices_Serialization.hpp>

namespace surface_types {
struct SurfaceData {
    uint32_t id;
    std_msgs::ColorRGBA color;
    pcl::PointCloud<pcl::PointXYZ> inliers;
    pcl::ModelCoefficients model;
    // An Eigen::Affine3d with a DontAlign option
    Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> pose;
    pcl::PointCloud<pcl::PointXYZ> boundary;
    std::vector<pcl::Vertices> polygons;
    shape_msgs::Mesh mesh;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SurfaceData() : id(0), color(), inliers(), model(), pose(), boundary(), polygons(), mesh() {}
    SurfaceData(uint32_t i, std_msgs::ColorRGBA c, pcl::PointCloud<pcl::PointXYZ> p) :
            id(std::move(i)), color(std::move(c)), inliers(std::move(p)), model(), pose(), boundary(), polygons(), mesh() {}
    SurfaceData(uint32_t i, std_msgs::ColorRGBA c, pcl::PointCloud<pcl::PointXYZ> p, pcl::ModelCoefficients m, Eigen::Affine3d t) :
            id(std::move(i)), color(std::move(c)), inliers(std::move(p)), model(std::move(m)), pose(std::move(t)), boundary(), polygons(), mesh() {}

    operator surface_msgs2::Surface() const {
        // Can only convert complete surfaces to ROS messages
        validate_complete();

        surface_msgs2::Surface s_ros;

        // ID, color, model, pose
        s_ros.id = id;
        s_ros.color = color;
        pcl_conversions::fromPCL(model, s_ros.model);

        // The aligned-ness of the pose member makes inference fail, so make a quick copy
        // Technically inefficient but effect should be negligible
        Eigen::Affine3d pose_aligned(pose);
        tf::poseEigenToMsg(pose_aligned, s_ros.pose);

        // Inliers
        pcl::PCLPointCloud2 inliers_pointcloud2;
        pcl::toPCLPointCloud2<pcl::PointXYZ>(inliers, inliers_pointcloud2);
        pcl_conversions::fromPCL(inliers_pointcloud2, s_ros.inliers);

        // Boundary
        pcl::PCLPointCloud2 boundary_pointcloud2;
        pcl::toPCLPointCloud2<pcl::PointXYZ>(boundary, boundary_pointcloud2);
        pcl_conversions::fromPCL(boundary_pointcloud2, s_ros.boundary);

        // Vertices
        s_ros.polygons.resize(polygons.size());
        for (std::size_t i = 0; i < polygons.size(); i++) {
            pcl_conversions::fromPCL(polygons[i], s_ros.polygons[i]);
        }

        // Mesh
        s_ros.mesh = mesh;

        return s_ros;
    }

    void validate() const {
        if (is_complete()) {
            validate_complete(); // Reduntant, but keeping it in case I remove the check from is_complete
        } else {
            validate_partial();
        }
    }

    void validate_complete() const {
        // ID and color technically have no invalid values
        assert(inliers.size() > 0 && "Attempted to validate a surface with no inliers");
        assert(model.values.size() == 4 &&
               "Attempted to validate a surface with an invalid model (to few or too many values)");
        assert(std::abs(Eigen::Map<const Eigen::Vector3f>(model.values.data()).norm() - 1) < 1e-3 &&
               "Attempted to validate a surface with an invalid model (normal is not unit length)");
        // pose can't be invalid (I think)

        assert(boundary.size() > 0 && "Attempted to validate a complete surface with no boundary");
        assert(polygons.size() > 0 && "Attempted to validate a complete surface with no polygons");
        assert(std::all_of(polygons.begin(), polygons.end(), [](pcl::Vertices v) { return v.vertices.size() > 0; }) &&
               "Attempted to validate a complete surface with an empty polygon");
        assert(mesh.vertices.size() > 0 && "Attempted to validate a complete surface with an empty mesh (no vertices)");
        assert(mesh.triangles.size() > 0 &&
               "Attempted to validate a complete surface with an empty mesh (no triangles)");
    }

    void validate_partial() const {
        // ID and color technically have no invalid values
        assert(inliers.size() > 0 && "Attempted to validate a surface with no inliers");
        assert(model.values.size() == 4 &&
               "Attempted to validate a surface with an invalid model (to few or too many values)");
        assert(std::abs(Eigen::Map<const Eigen::Vector3f>(model.values.data()).norm() - 1) < 1e-3 &&
               "Attempted to validate a surface with an invalid model (normal is not unit length)");
        // pose can't be invalid (I think)

        assert(boundary.size() == 0 && "Attempted to validate a partial surface with non-empty boundary");
        assert(polygons.size() == 0 && "Attempted to validate a partial surface with non-empty polygons");
        // NOTE I now allow meshes in partial surfaces because of mesh re-use
//        assert(mesh.vertices.size() == 0 && "Attempted to validate a partial surface with non-empty mesh (vertices)");
//        assert(mesh.triangles.size() == 0 && "Attempted to validate a partial surface with non-empty mesh (triangles)");
    }

    bool is_complete() const {
        // Use boundary.size as the indicator of completeness
        if (boundary.size() > 0) {
            validate_complete();

            return true;
        } else {
            validate_partial();

            return false;
        }
    }

    void clear_computed_values() {
        boundary.clear();
        polygons.clear();
        mesh = shape_msgs::Mesh();
    }

    typedef boost::shared_ptr<::surface_types::SurfaceData> Ptr;
    typedef boost::shared_ptr<::surface_types::SurfaceData const> ConstPtr;
};

// struct SurfaceData

inline std::ostream &operator<<(std::ostream &s, const SurfaceData &v) {
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
    if (v.is_complete()) {
        // Print eigen transform as a matrix using .format() to set the indentation
        s << v.pose.matrix().format(Eigen::IOFormat(Eigen::StreamPrecision, 0, " ", "\n", "  ")) << std::endl;
        s << "boundary[" << v.boundary.size() << "]:" << std::endl;
        for (auto &inlier : v.boundary)
            s << "  " << inlier << std::endl;
        s << "polygons[" << v.polygons.size() << "]:" << std::endl;
        for (auto &vertices : v.polygons) {
            s << "  vertices[" << vertices.vertices.size() << "]:" << std::endl;
            for (auto &vertex : vertices.vertices)
                s << "    " << vertex << std::endl;
        }
        s << "mesh:" << std::endl;
        s << "  " << v.mesh << std::endl; // You're on your own for this one
    }
    return (s);
}
}

namespace ros {
namespace message_traits {
template <>
struct IsFixedSize<surface_types::SurfaceData> : public FalseType {};
template <>
struct IsSimple<surface_types::SurfaceData> : public FalseType {};
template <>
struct HasHeader<surface_types::SurfaceData> : public FalseType {};

template <>
struct MD5Sum<surface_types::SurfaceData> {
    static const char *value() { return MD5Sum<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::SurfaceData &m) { return MD5Sum<surface_msgs2::Surface>::value(); }
};

template <>
struct DataType<surface_types::SurfaceData> {
    static const char *value() { return DataType<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::SurfaceData &m) { return DataType<surface_msgs2::Surface>::value(); }
};

template <>
struct Definition<surface_types::SurfaceData> {
    static const char *value() { return Definition<surface_msgs2::Surface>::value(); }

    static const char *value(const surface_types::SurfaceData &m) { return Definition<surface_msgs2::Surface>::value(); }
};
} // namespace message_traits

namespace serialization {
template <>
struct Serializer<surface_types::SurfaceData> {
    template <typename Stream>
    inline static void write(Stream &stream, const surface_types::SurfaceData &s) {
        // Only try to serialize complete surfaces
        s.validate_complete();

        geometry_msgs::Pose geom_pose;
        // The aligned-ness of the pose member makes inference fail, so make a quick copy
        // Technically inefficient but effect should be negligible
        Eigen::Affine3d pose_aligned(s.pose);
        tf::poseEigenToMsg(pose_aligned, geom_pose);

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
    inline static void read(Stream &stream, surface_types::SurfaceData &s) {
        geometry_msgs::Pose geom_pose;

        stream.next(s.id);
        stream.next(s.color);
        stream.next(s.model);
        stream.next(s.inliers);
        stream.next(geom_pose);
        stream.next(s.boundary);
        stream.next(s.polygons);
        stream.next(s.mesh);

        // The aligned-ness of the pose member makes inference fail, so make a quick copy
        // Technically inefficient but effect should be negligible
        Eigen::Affine3d pose_aligned;
        tf::poseMsgToEigen(geom_pose, pose_aligned);
        s.pose = pose_aligned;
    }

    inline static uint32_t serializedLength(const surface_types::SurfaceData &s) {
        geometry_msgs::Pose geom_pose;
        // The aligned-ness of the pose member makes inference fail, so make a quick copy
        // Technically inefficient but effect should be negligible
        Eigen::Affine3d pose_aligned(s.pose);
        tf::poseEigenToMsg(pose_aligned, geom_pose);

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
