//
// Created by will on 6/21/16.
//

#include "expand_surfaces/ExpandSurfaces.h"

// std and Boost
#include <queue>

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// SurfaceData types
#include <surface_types/Surface.hpp>

// Utils
#include <surface_utils/smart_ptr.hpp>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>

ExpandSurfaces::ExpandSurfaces(double perpendicular_dist, double parallel_dist, double disc)
    : perpendicular_distance_(perpendicular_dist), parallel_distance_(parallel_dist), discretization_(disc) {
    assert(perpendicular_distance_ > 0);
    assert(parallel_distance_ > 0);
}

std::vector<int> ExpandSurfaces::expandAlongPlane(const PointCloud &cloud, const Search &search,
                                                  const PointCloud &edge_points, const Eigen::Affine3f &tf) const {
    // processed[i] indicates whether point i has been added to the queue
    std::vector<int> processed(cloud.size(), -1);
    // Don't have a label in this version, but only one label is ever used, so any arbitrary value works (here 0)
    return expandAlongPlane(cloud, search, edge_points, tf, processed, 0);
}

std::vector<int> ExpandSurfaces::expandAlongPlane(const ExpandSurfaces::PointCloud &cloud,
                                                  const ExpandSurfaces::Search &search,
                                                  const ExpandSurfaces::PointCloud &edge_points,
                                                  const Eigen::Affine3f &tf, std::vector<int> &processed,
                                                  const uint32_t label) const {
    pcl::ScopeTime st("expandAlongPlane");
    double set_time = 0, kdtree_time = 0, tf_time = 0, total_time = -ros::WallTime::now().toSec();

    // Note these are (or should be) cleared by radiusSearch on each loop iteration
    std::vector<std::vector<int>> neighbor_indices;
    std::vector<std::vector<float>> sqrdistances;

    std::vector<int> to_search;

    auto nn_start = ros::WallTime::now();
    // Seed the search with the 1nn of every point in edge_points
    // Setting processed[i] for each 1nn prevents considering every member of edge_points twice!
    kdtree_time -= ros::WallTime::now().toSec();
    search.nearestKSearch(edge_points, {}, 1, neighbor_indices, sqrdistances);
    kdtree_time += ros::WallTime::now().toSec();
    for (std::size_t i = 0; i < edge_points.size(); i++) {
        if (sqrdistances[i][0] > this->parallel_distance_ * this->parallel_distance_) {
            // Then there is no corresponding neighbor for this point
            continue;
        } else if (processed[neighbor_indices[i][0]] >= 0) {
            // Then the corresponding neighbor was already claimed by another surface
            continue;
        }
        to_search.push_back(neighbor_indices[i][0]);
        set_time -= ros::WallTime::now().toSec();
        processed[neighbor_indices[i][0]] = static_cast<int>(label);
        set_time += ros::WallTime::now().toSec();
    }
    ROS_DEBUG_STREAM("1nn search in expandAlongPlane took " << (ros::WallTime::now() - nn_start).toSec() << "s");

    auto tfi = tf.inverse();

    // TODO: Come up with a more principled way to find this threshold
    // (Maybe it should be that the spread along smallest axis other than the normal's axis is less than perp. dist?)
    int req_no_points = static_cast<int>(std::ceil((this->parallel_distance_ / this->discretization_) *
                                                   (this->perpendicular_distance_ / this->discretization_)));

    while (!to_search.empty()) {
        kdtree_time -= ros::WallTime::now().toSec();
        search.radiusSearch(cloud, to_search, this->parallel_distance_, neighbor_indices, sqrdistances);
        kdtree_time += ros::WallTime::now().toSec();

        to_search.clear();
        for (std::size_t i = 0; i < neighbor_indices.size(); i++) {
            // NOTE after this call, indices_end should be used instead of neighbor_indices.end()
            // (this avoids the need to call .erase(), since this container is reset by radiusSearch on the next
            // iteration)
            auto indices_end = std::remove_if(neighbor_indices[i].begin(), neighbor_indices[i].end(), [&](int idx) {
                set_time -= ros::WallTime::now().toSec();
                bool found = processed[idx] > 0;
                set_time += ros::WallTime::now().toSec();
                if (found) return found;

                tf_time -= ros::WallTime::now().toSec();
                bool outside = std::abs(pcl::transformPoint(cloud[idx], tfi).z) > this->perpendicular_distance_;
                tf_time += ros::WallTime::now().toSec();

                return outside;
            });

            // If there aren't enough found points, that indicates we're following a very thin section, which is
            // probably
            // part of another surface, so don't add any of these neighbors
            if (std::distance(neighbor_indices[i].begin(), indices_end) < req_no_points) {
                continue;
            }

            // In this loop, every element has already passed the bounds and duplicates check
            for (auto iter = neighbor_indices[i].begin(); iter != indices_end; ++iter) {
                set_time -= ros::WallTime::now().toSec();
                processed[*iter] = static_cast<int>(label);
                set_time += ros::WallTime::now().toSec();
                to_search.push_back(*iter);
            }
        }
    }

    total_time += ros::WallTime::now().toSec();

    ROS_DEBUG_STREAM(std::fixed << std::setprecision(1) << "expandAlongPlane: " << set_time / 1000.
                                << "ms set operations, " << kdtree_time / 1000. << "ms kd-tree search, "
                                << tf_time / 1000. << "ms point transforms, " << total_time / 1000. << " total");

    std::vector<int> within_radius_indices;
    for (std::size_t i = 0; i < processed.size(); i++) {
        if (processed[i] == static_cast<int>(label)) {
            within_radius_indices.push_back(static_cast<int>(i));
        }
    }

    ROS_DEBUG_STREAM("got " << within_radius_indices.size() << " points (started with " << edge_points.size() << ")");
    return within_radius_indices;
}

void ExpandSurfaces::expand_surfaces(const std::vector<Surface> &surfaces, const PointCloud &cloud,
                                     std::vector<int> &indices, std::function<void(Surface)> callback) {
    // pcl::ScopeTime st("SurfaceDetection::detect_surfaces_within");

    // If no points or no indices or no surfaces, then no work to do
    if (cloud.empty() || indices.empty() || surfaces.empty()) {
        return;
    }

    pcl::search::KdTree<Point> search(false);
    search.setInputCloud(boost_fake_shared(cloud), boost_fake_shared(indices));

    // This vector acts as a map from point index to the surface
    std::vector<int> in_surface(cloud.size(), -1);

    for (auto &old_surface : surfaces) {
        // Get the new indices, while simultaneously updating in_surface
        auto new_indices = expandAlongPlane(cloud, search, old_surface.boundary_or_inliers(), old_surface.pose_float(),
                                            in_surface, old_surface.id());
        if (new_indices.size() == 0) continue;

        Surface new_surface = old_surface;
        new_surface.add_inliers(cloud, indices);

        callback(new_surface);
    }

    // Remove any points which were given to a surface to `indices`
    indices.erase(std::remove_if(indices.begin(), indices.end(), [&in_surface](const int idx) {
        return in_surface[idx] > 0;
    }), indices.end());
}

void ExpandSurfaces::expand_surface(const PointCloud &points, const pcl::search::Search<Point> &search,
                                        const Surface &prev_size, std::function<void(std::vector<int>)> callback) {
    auto new_indices = expandAlongPlane(points, search, prev_size.boundary_or_inliers(), prev_size.pose_float());

    callback(new_indices);
}
