//
// Created by will on 6/21/16.
//

#include "expand_surfaces/ExpandSurfaces.h"

// std and Boost
#include <queue>

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// Surface types
#include <surface_types/Surface.hpp>

// Utils
#include <surface_utils/smart_ptr.hpp>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>

ExpandSurfaces::ExpandSurfaces(double perpendicular_dist, double parallel_dist, double disc)
    : perpendicular_distance_(perpendicular_dist), parallel_distance_(parallel_dist), discretization_(disc) {
    assert(perpendicular_distance_ > 0);
    assert(parallel_distance_ > 0);
}

std::set<int> ExpandSurfaces::filterWithinRadiusConnected(const PointCloud &cloud, const Search &search,
                                                          const PointCloud &edge_points,
                                                          const Eigen::Affine3f &tf) const {
    pcl::ScopeTime st("filterWithinRadiusConnected");
    std::set<int> within_radius_indices;

    double set_time = 0, kdtree_time = 0, tf_time = 0, total_time = -ros::WallTime::now().toSec();


    std::queue<Point> to_search(std::deque<Point>(edge_points.begin(), edge_points.end()));

    auto tfi = tf.inverse();

    // TODO: Come up with a more principled way to find this threshold
    // (Maybe it should be that the spread along smallest axis other than the normal's axis is less than perp. dist?)
    int req_no_points = static_cast<int>(
        std::ceil((parallel_distance_ / discretization_) * (perpendicular_distance_ / discretization_))) * 2;

    // Note these are (should be) cleared by radiusSearch on each loop iteration
    std::vector<int> neighbor_indices;
    std::vector<float> tmp_sqrdistances; // Only needed to fill a parameter

    while (!to_search.empty()) {
        const auto &point = to_search.front();

        kdtree_time -= ros::WallTime::now().toSec();
        search.radiusSearch(point, parallel_distance_, neighbor_indices, tmp_sqrdistances);
        kdtree_time += ros::WallTime::now().toSec();

        // Don't delete the point until after it's used because `point` is a reference to it
        to_search.pop();

        // NOTE after this all, indices_end should be used instead of neighbor_indices.end()
        // (this avoids the need to call .erase(), since this container is reset by radiusSearch on the next iteration)
        auto indices_end = std::remove_if(neighbor_indices.begin(), neighbor_indices.end(), [&](int i) {
            set_time -= ros::WallTime::now().toSec();
            bool found = within_radius_indices.find(i) != within_radius_indices.end();
            set_time += ros::WallTime::now().toSec();
            if (found) return found;

            tf_time -= ros::WallTime::now().toSec();
            bool outside = std::abs(pcl::transformPoint(cloud[i], tfi).z) > perpendicular_distance_;
            tf_time += ros::WallTime::now().toSec();

            return outside;
        });

        // If there aren't enough found points, that indicates we're following a very thin section, which is probably
        // part of another surface, so don't add any of these neighbors
        if (std::distance(neighbor_indices.begin(), indices_end) < req_no_points) { // TODO magic number
            continue;
        }

        // In this loop, every element has already passed the bounds and duplicates check
        for (auto iter = neighbor_indices.begin(); iter != indices_end; ++iter) {
            set_time -= ros::WallTime::now().toSec();
            within_radius_indices.insert(*iter);
            set_time += ros::WallTime::now().toSec();
            to_search.push(search.getInputCloud()->at(static_cast<std::size_t>(*iter)));
        }
    }

    total_time += ros::WallTime::now().toSec();

    ROS_DEBUG_STREAM(std::fixed << std::setprecision(1) << "filterWithinRadiusConnected: " << set_time / total_time * 100
                     << "% set operations, " << kdtree_time / total_time * 100 << "% kd-tree search, "
                     << tf_time / total_time * 100 << "% point transforms");

    return within_radius_indices;
}

pcl::PointIndices ExpandSurfaces::filterWithinRadiusConnected(const PointCloud &cloud, const Search &search,
                                                              const PointCloud &edge_points, const Eigen::Affine3f &tf,
                                                              const pcl::PointIndices &remaining_indices) const {
    std::set<int> within_radius_indices = filterWithinRadiusConnected(cloud, search, edge_points, tf);

    pcl::PointIndices within_radius_nodupes;
    // This may reserve more space than necessary, but it shouldn't be much more.
    within_radius_nodupes.indices.reserve(within_radius_indices.size());

    // Sortedness Invariants:
    // within_radius_indices is sorted because it's a set
    // remaining_indices' sortedness is a loop invariant
    std::set_intersection(within_radius_indices.begin(), within_radius_indices.end(), remaining_indices.indices.begin(),
                          remaining_indices.indices.end(), std::back_inserter(within_radius_nodupes.indices));

    return within_radius_nodupes;
}

pcl::PointIndices ExpandSurfaces::filterWithinModelDistance(const PointCloud::ConstPtr &input,
                                                            const pcl::PointIndices &indices,
                                                            const pcl::ModelCoefficients &coeff) {
    auto model = pcl::SampleConsensusModelPlane<Point>(input, indices.indices);

    pcl::PointIndices output_indices;
    model.selectWithinDistance(Eigen::Vector4f(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]),
                               perpendicular_distance_, output_indices.indices);

    return output_indices;
}

pcl::PointIndices ExpandSurfaces::expand_surfaces(const std::vector<Surface> &surfaces, const CloudIndexPair &input,
                                                  std::function<void(Surface)> callback) {
    // pcl::ScopeTime st("SurfaceDetection::detect_surfaces_within");
    auto cloud = boost::shared_ptr<const PointCloud>(&input.first, null_deleter());
    auto indices = boost::shared_ptr<const pcl::PointIndices>(&input.second, null_deleter());

    if (cloud->size() == 0 || indices->indices.size() == 0 || surfaces.size() == 0) {
        return input.second;
    }

    pcl::search::KdTree<Point> search(false);
    search.setInputCloud(cloud, boost::shared_ptr<const std::vector<int>>(&input.second.indices, null_deleter()));

    pcl::PointIndices remaining_indices;
    pcl::PointIndices remaining_indices_tmp;
    remaining_indices.indices = indices->indices;

    for (const auto &old_surface : surfaces) {

        //
        // FILTER TO POINTS NEAR SURFACE BOUNDARY AND ON THE SURFACE
        //
        auto distance_filtered = filterWithinRadiusConnected(*cloud, search, old_surface.boundary,
                                                             old_surface.pose.cast<float>(), remaining_indices);
        if (distance_filtered.indices.size() == 0) continue;

        //
        // ADD NEW POINTS TO SURFACE
        //
        surface_types::Surface new_surface(old_surface);
        new_surface.inliers += pcl::PointCloud<Point>(*cloud, distance_filtered.indices);
        new_surface.clear_computed_values();

        callback(new_surface);

        //
        // REMOVE THOSE POINTS FROM THE INPUT
        //
        remaining_indices_tmp.indices.clear();
        remaining_indices_tmp.indices.reserve(remaining_indices.indices.size() - distance_filtered.indices.size());
        std::set_difference(remaining_indices.indices.begin(), remaining_indices.indices.end(),
                            distance_filtered.indices.begin(), distance_filtered.indices.end(),
                            std::back_inserter(remaining_indices_tmp.indices));
        remaining_indices.indices.swap(remaining_indices_tmp.indices);
    }

    // Always publish remaining indices
    return remaining_indices;
}

void ExpandSurfaces::expand_new_surface(const PointCloud &points, const pcl::search::Search<Point> &search,
                                        const Surface &new_surface, std::function<void(pcl::PointIndices)> callback) {
    //
    // FILTER TO POINTS NEAR SURFACE BOUNDARY
    //
    auto radius_filtered =
        filterWithinRadiusConnected(points, search, new_surface.inliers, new_surface.pose.cast<float>());

    //
    // ADD NEW POINTS TO SURFACE
    //
    pcl::PointIndices indices;
    indices.indices.reserve(indices.indices.size() + radius_filtered.size());
    std::copy(radius_filtered.begin(), radius_filtered.end(), std::back_inserter(indices.indices));

    callback(indices);
}
