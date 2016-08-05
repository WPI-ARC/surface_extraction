//
// Created by will on 6/21/16.
//

#include "expand_surfaces/ExpandSurfaces.h"

// std and Boost
#include <queue>
#include <chrono>

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/distances.h>

// SurfaceData types
#include <surface_types/Surface.hpp>

// Utils
#include <surface_utils/smart_ptr.hpp>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <surface_utils/SurfaceVisualizationController.hpp>

ExpandSurfaces::ExpandSurfaces(double perpendicular_dist, double parallel_dist, double disc)
    : perpendicular_distance_(perpendicular_dist), parallel_distance_(parallel_dist), discretization_(disc) {
    assert(perpendicular_distance_ > 0);
    assert(parallel_distance_ > 0);
}

std::vector<int> ExpandSurfaces::expandAlongPlane(const ExpandSurfaces::PointCloud &cloud,
                                                  const ExpandSurfaces::PointCloud &edge_points_pre_tf,
                                                  const pcl::ModelCoefficients &model, const Eigen::Affine3f &tf,
                                                  std::vector<int> &processed,
                                                  std::set<std::pair<uint32_t, uint32_t>> &merge_candidates,
                                                  const uint32_t label, const SurfaceVisualizationController &v) const {
    // Note these are (or should be) cleared by radiusSearch on each loop iteration
    std::vector<int> neighbor_indices;
    std::vector<float> sqrdistances;
    std::vector<int> query_pts;

    auto tf_time_start = std::chrono::high_resolution_clock::now();
    PointCloud cloud_in_plane;
    pcl::transformPointCloud(cloud, cloud_in_plane, tf.inverse());
    std::vector<int> near_plane;
    std::vector<int> in_plane;
    for (std::size_t i = 0; i < cloud_in_plane.size(); i++) {
        if (std::abs(cloud_in_plane[i].z) > parallel_distance_) {
            continue;
        } else if (std::abs(cloud_in_plane[i].z) > perpendicular_distance_) {
            near_plane.push_back(static_cast<int>(i));
        } else if (processed[i] < 0) {
            in_plane.push_back(static_cast<int>(i));
        }
    }

    if (in_plane.empty()) {
        return query_pts;
    }

    pcl::search::KdTree<Point> in_plane_search(false);
    in_plane_search.setInputCloud(boost_fake_shared(cloud_in_plane), boost_fake_shared(in_plane));
    pcl::search::KdTree<Point> near_plane_search(false);
    if (!near_plane.empty()) {
        near_plane_search.setInputCloud(boost_fake_shared(cloud_in_plane), boost_fake_shared(near_plane));
    }

    PointCloud edge_points;
    pcl::transformPointCloud(edge_points_pre_tf, edge_points, tf.inverse());

    ROS_DEBUG_STREAM("Took " << std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::high_resolution_clock::now() - tf_time_start).count()
                             << "ms to transform " << cloud.size() << " points and filter to " << in_plane.size()
                             << " with " << near_plane.size() << " reject indicators");

    // Seed the in_plane_search with the 1nn of every point in edge_points
    // Setting processed[i] for each 1nn prevents considering every member of edge_points twice
    neighbor_indices.resize(1);
    sqrdistances.resize(1);
    auto sqr_parallel_distance = parallel_distance_ * parallel_distance_;
    for (const auto edge_pt : edge_points) {
        in_plane_search.nearestKSearch(edge_pt, 1, neighbor_indices, sqrdistances);
        if (sqrdistances[0] <= sqr_parallel_distance) {
            if (processed[neighbor_indices[0]] < 0) {
                processed[neighbor_indices[0]] = static_cast<int>(label);
                query_pts.push_back(neighbor_indices[0]);
            } else if (processed[neighbor_indices[0]] != label) {
                merge_candidates.insert(std::minmax(label, static_cast<uint32_t>(processed[neighbor_indices[0]])));
            }
        }
    }
    ROS_DEBUG_STREAM("1nn in_plane_search in expandAlongPlane found " << query_pts.size()
                                                                      << " points to in_plane_search");

    std::size_t current_query_pt = 0;
    while (current_query_pt < query_pts.size()) {
        // Don't just provide query_pts[current_query_pt] to searches -- it interprets that as an index into `in_plane`
        Point query_pt = cloud_in_plane[query_pts[current_query_pt]];
        query_pt.z = 0;
        current_query_pt++;

        // Limit the radius to either parallel_distance OR whatever distance excludes the nearest reject indicator
        double radius = parallel_distance_;
        if (!near_plane.empty()) {
            neighbor_indices.resize(1);
            sqrdistances.resize(1);
            near_plane_search.nearestKSearch(query_pt, 1, neighbor_indices, sqrdistances);
            if (sqrdistances[0] <= sqr_parallel_distance) {
                auto dx = query_pt.x - cloud_in_plane[neighbor_indices[0]].x;
                auto dy = query_pt.y - cloud_in_plane[neighbor_indices[0]].y;
                double projected_dist = std::sqrt(dx * dx + dy * dy);
                radius = std::min(radius, projected_dist);
            }
        }

        in_plane_search.radiusSearch(query_pt, radius, neighbor_indices, sqrdistances);

        // Remove indices which are already processed and ones which are too far away
        // This breaks the association between neighbor_indices[i][j] and sqrdistances[i][j]
        neighbor_indices.erase(std::remove_if(neighbor_indices.begin(), neighbor_indices.end(), [&](int idx) {
            if (processed[idx] < 0) {
                return false;
            } else if (processed[idx] != label) {
                merge_candidates.insert(std::minmax(label, static_cast<uint32_t>(processed[idx])));
            }
            return true;
        }), neighbor_indices.end());

        //        // Compute centroid
        //        Eigen::Vector4f centroid;
        //        pcl::compute3DCentroid(cloud_in_plane, neighbor_indices, centroid);
        //
        //        // Compute eigenvectors and eigenvalues
        //        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        //        pcl::computeCovarianceMatrix(cloud_in_plane, neighbor_indices, centroid, covariance_matrix);
        //        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        //        float eigen_value;
        //        pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);
        //
        //        if (pcl::getAngle3D(Eigen::Vector3f::UnitZ(), eigen_vector) > 0.2) {
        //            normal_pts.clear();
        //            // This is very definitely just for debugging purposes
        //            for (auto idx : neighbor_indices) {
        //                pcl::PointNormal npt;
        //                npt.x = cloud_in_plane[idx].x;
        //                npt.y = cloud_in_plane[idx].y;
        //                npt.z = cloud_in_plane[idx].z;
        //                npt.normal_x = eigen_vector[0];
        //                npt.normal_y = eigen_vector[1];
        //                npt.normal_z = eigen_vector[2];
        //                normal_pts.push_back(npt);
        //            }
        //            auto normal_points_transformed = boost::make_shared<decltype(normal_pts)>();
        //            pcl::transformPointCloudWithNormals(normal_pts, *normal_points_transformed, tf);
        //            v.normal_vectors("rejected_expansions", normal_points_transformed);
        //            v.points<pcl::PointNormal>("rejected_expansion_pts", normal_points_transformed);
        //            ros::WallDuration(1).sleep();
        //            continue;
        //        }

        // In this loop, every element has already passed the bounds and duplicates check
        for (auto idx : neighbor_indices) {
            processed[idx] = static_cast<int>(label);
            query_pts.push_back(idx);
        }
    }

    // query_pts is exactly equal to the set of points that was newly labeled with label
    return query_pts;
}
