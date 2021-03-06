//
// Created by will on 3/16/16.
//

#ifndef SURFACES_UTILS_HPP
#define SURFACES_UTILS_HPP

#include <ros/console.h>
#include <boost/smart_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/range/join.hpp>

#include <Eigen/Geometry>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/exceptions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl_msgs/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>

namespace surfaces_pcl_utils {
    struct PCLHeaderHashNoSeq {
        std::size_t operator()(const pcl::PCLHeader &h) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, h.frame_id);
            boost::hash_combine(seed, h.stamp);
            return seed;
        };
    };

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
        auto result = Eigen::Translation3f(0, 0, d) * arot.inverse();

        return result;
    }

    inline Eigen::Affine3f tf_from_plane_model(const pcl_msgs::ModelCoefficients &plane) {
        return tf_from_plane_model(plane.values[0], plane.values[1], plane.values[2], plane.values[3]);
    }

    inline Eigen::Affine3f tf_from_plane_model(const pcl::ModelCoefficients &plane) {
        return tf_from_plane_model(plane.values[0], plane.values[1], plane.values[2], plane.values[3]);
    }

    template <typename T>
    inline pcl::IndicesPtr all_indices(const typename pcl::PointCloud<T>::ConstPtr cloud) {
        pcl::IndicesPtr indices = boost::make_shared<std::vector<int> >();
        indices->resize(cloud->size());
        std::iota(indices->begin(), indices->end(), 0);
        return indices;
    }

    template <typename T>
    inline std::vector<int> all_indices(const typename pcl::PointCloud<T> &cloud) {
        std::vector<int> indices;
        indices.resize(cloud.size());
        std::iota(indices.begin(), indices.end(), 0);
        return indices;
    }

    inline std::size_t count_vertices(const std::vector<pcl::Vertices> &polygons) {
        return std::accumulate(polygons.begin(), polygons.end(), std::size_t(0),
                               [](const std::size_t sum, const pcl::Vertices &v){ return sum + v.vertices.size(); });
    }

    inline std::vector<int> reindex(const std::vector<int> &indexer, const std::vector<int> &indices) {
        // Takes indices that relate to an original cloud and another set that relates to a cloud that was filtered
        // by the original and returns an indices set with the contents of the second that relates to the original cloud
        // (Turns cloud[indexer[indices[i]]] into cloud[reindexed[i]] for i = 0..indices.size())
        if (indexer.size() <= indices.size()) {
            ROS_WARN("Indices is not smaller than reindexer, which is only valid if indices contains duplicates");
        }
        std::vector<int> reindexed;
        for (auto index : indices) {
            assert(index >= 0);

            if (static_cast<std::size_t>(index) >= indexer.size()) {
                ROS_ERROR_STREAM("Error: index " << index << " is out of bounds of indexer");
            }

            // Use .at to get the bounds check
            reindexed.push_back(indexer.at(static_cast<std::size_t>(index)));
        }
        return reindexed;
    }

    inline pcl::PointIndices reindex(const pcl::PointIndices &indexer, const pcl::PointIndices &indices) {
        // Takes a PointIndices that relates to an original cloud and another that relates to a cloud that was filtered
        // by the original and returns a PointIndices with the contents of the second that relates to the original cloud
        // (Turns cloud[indexer[indices[i]]] into cloud[reindexed[i]] for i = 0..indices.size())
        pcl::PointIndices reindexed;
        reindexed.header = indices.header;
        reindexed.indices = reindex(indexer.indices, indices.indices);
        return reindexed;
    }
}

#endif //SURFACES_UTILS_HPP
