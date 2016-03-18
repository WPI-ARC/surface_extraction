//
// Created by will on 3/16/16.
//

#ifndef SURFACES_UTILS_HPP
#define SURFACES_UTILS_HPP

#include <boost/lexical_cast.hpp>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/exceptions.h>
#include <Eigen/Geometry>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl_msgs/PointIndices.h>

#define tri_x(points, i) (points[i * 2])
#define tri_y(points, i) (points[i * 2 + 1])
#define tri_v1(triangles, i) (static_cast<uint32_t>(triangles[i * 3]))
#define tri_v2(triangles, i) (static_cast<uint32_t>(triangles[i * 3 + 1]))
#define tri_v3(triangles, i) (static_cast<uint32_t>(triangles[i * 3 + 2]))


namespace surfaces {
    inline pcl::SacModel sacModelFromConfigInt(int config_model_type) {
        switch (config_model_type) {
            case 0:
                return pcl::SACMODEL_PLANE;
            case 1:
                return pcl::SACMODEL_LINE;
            case 2:
                return pcl::SACMODEL_CIRCLE2D;
            case 3:
                return pcl::SACMODEL_CIRCLE3D;
            case 4:
                return pcl::SACMODEL_SPHERE;
            case 5:
                return pcl::SACMODEL_CYLINDER;
            case 6:
                return pcl::SACMODEL_CONE;
            case 7:
                return pcl::SACMODEL_TORUS;
            case 8:
                return pcl::SACMODEL_PARALLEL_LINE;
            case 9:
                return pcl::SACMODEL_PERPENDICULAR_PLANE;
            case 10:
                return pcl::SACMODEL_PARALLEL_LINES;
            case 11:
                return pcl::SACMODEL_NORMAL_PLANE;
            case 12:
                return pcl::SACMODEL_NORMAL_SPHERE;
            case 13:
                return pcl::SACMODEL_REGISTRATION;
            case 14:
                return pcl::SACMODEL_REGISTRATION_2D;
            case 15:
                return pcl::SACMODEL_PARALLEL_PLANE;
            case 16:
                return pcl::SACMODEL_NORMAL_PARALLEL_PLANE;
            case 17:
                return pcl::SACMODEL_STICK;
            default:
                throw pcl::InvalidSACModelTypeException(
                        "No SAC model for integer " + boost::lexical_cast<std::string>(config_model_type));
        }
    }

    inline Eigen::Affine3f tf_from_plane_model(float a, float b, float c, float d) {
        Eigen::Vector3f new_normal(a, b, c);
        Eigen::Vector3f old_normal(0, 0, 1);

        Eigen::Vector3f v = old_normal.cross(new_normal);
        float s2 = v.squaredNorm();
        float cc = old_normal.dot(new_normal);
        Eigen::Matrix3f v_cross;
        v_cross << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

        Eigen::Matrix3f rot = Eigen::Matrix3f::Identity() + v_cross + v_cross * v_cross * (1 - cc) / s2;
        Eigen::Affine3f arot(rot);

        // Create a transform where the rotation component is given by the rotation axis as the normal vector (a, b, c)
        // and some arbitrary angle about that axis and the translation component is -d in the z direction after
        // that rotation (not the original z direction, which is how transforms are usually defined).
        return arot * Eigen::Translation3f(0, 0, -d);
    }

    inline Eigen::Affine3f tf_from_plane_model(const pcl_msgs::ModelCoefficients &plane) {
        return tf_from_plane_model(plane.values[0], plane.values[1], plane.values[2], plane.values[3]);
    }

    inline Eigen::Affine3f tf_from_plane_model(const pcl::ModelCoefficients &plane) {
        return tf_from_plane_model(plane.values[0], plane.values[1], plane.values[2], plane.values[3]);
    }

    template <typename T>
    inline pcl_msgs::PointIndices::ConstPtr all_indices(typename pcl::PointCloud<T>::ConstPtr cloud) {
        pcl_msgs::PointIndices::Ptr indices = boost::make_shared<pcl_msgs::PointIndices>();
        pcl_conversions::fromPCL(cloud->header, indices->header);
        indices->indices.resize(cloud->size());
        std::iota(indices->indices.begin(), indices->indices.end(), 0);
        return indices;
    }

}

#endif //SURFACES_UTILS_HPP
