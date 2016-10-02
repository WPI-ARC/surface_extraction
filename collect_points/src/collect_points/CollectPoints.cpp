//
// Created by will on 6/21/16.
//

#include "collect_points/CollectPoints.h"

#include <ros/console.h>

// PCL Includes
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/passthrough.h>

// Utils
#include <arc_utilities/pretty_print.hpp>
#include <surface_utils/conversions.hpp>
#include <surface_utils/smart_ptr.hpp>
#include <pcl/common/time.h>
#include <eigen_conversions/eigen_msg.h>

using surface_utils::toLabeledPoint;

CollectPoints::CollectPoints(double discretization, double perpendicular_dist, double point_inside_threshold,
                             std::string target_frame, std::string camera_frame)
    : perpendicular_dist_(perpendicular_dist), point_inside_threshold_(point_inside_threshold),
      target_frame_(target_frame), camera_frame_(camera_frame), tf_listener_(), surface_points_(discretization),
      pending_points_(discretization) {
    pending_points_cloud_ = boost::make_shared<PointCloud>();
    pending_points_.setInputCloud(pending_points_cloud_);

    surface_points_cloud_ = boost::make_shared<LabeledCloud>();
    surface_points_.setInputCloud(surface_points_cloud_);
}

void CollectPoints::add_points(const PointCloud::ConstPtr &points) {
    ROS_INFO_STREAM_ONCE("Recieved first point cloud (" << points->size() << ")");
    ROS_INFO_NAMED("add_points", "Starting add_points");
    for (const auto &point : *points) {
        if (!inside_any_surface(point)) {
            pending_points_.addPointToCloud(point, pending_points_cloud_);
        }
    }
    ROS_INFO_NAMED("add_points", "Finished add_points");
}

CollectPoints::PointCloud CollectPoints::get_pending_points() {
    PointCloud cloud;
    // Assume the current camera transform is the viewpoint of all the pending points
    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(target_frame_, camera_frame_, ros::Time(0), transform);
    } catch (tf2::TransformException ex) {
        // I don't really want this throttled, but the only way I can get it to not warn within a few seconds of
        // starting up is to use DELAYED_THROTTLE, and it's probably fine if it is throttled
        ROS_WARN_STREAM_DELAYED_THROTTLE(5, "Failed to get camera viewpoint: " << ex.what());
        return cloud; // Return an empty cloud
    }
    // Populate the sensor origin
    cloud.sensor_origin_[0] = static_cast<float>(transform.getOrigin().x());
    cloud.sensor_origin_[1] = static_cast<float>(transform.getOrigin().y());
    cloud.sensor_origin_[2] = static_cast<float>(transform.getOrigin().z());
    cloud.sensor_origin_[3] = 1;
    // Populate the sensor orientation
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(transform.getRotation(), quat);
    Eigen::Quaterniond eigen_quat;
    tf::quaternionMsgToEigen(quat, eigen_quat);
    cloud.sensor_orientation_ = eigen_quat.cast<float>();

    // Octree has a bug where getVoxelCentroids segfaults when there are no voxels
    if (pending_points_.leaf_begin() == pending_points_.leaf_end()) {
        return cloud; // Return an empty cloud
    }
    pending_points_.getVoxelCentroids(cloud.points);
    return cloud;
}

bool CollectPoints::inside_any_surface(const Point &point) const {
    std::vector<int> neighbors;
    std::vector<float> distances;

    return surface_points_.radiusSearch(toLabeledPoint(point), point_inside_threshold_, neighbors, distances, 1) > 0;
}

// TODO refactor and/or rename to reflect the fact that this removes points within the extents, but not the padding
std::tuple<CollectPoints::CloudIndexPair, std::vector<int>>
CollectPoints::pending_points_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents, float padding)  {
    // pcl::ScopeTime st("CollectPoints::pending_points_within");
    auto result = std::make_pair(get_pending_points(), std::vector<int>());
    Eigen::Vector3f xyz, rpy;
    pcl::getTranslationAndEulerAngles(center, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);

    pcl::CropBox<Point> crop;

    crop.setInputCloud(boost_fake_shared(result.first));
    crop.setMax({extents[0] + padding, extents[1] + padding, extents[2] + padding, 1});
    crop.setMin({-(extents[0] + padding), -(extents[1] + padding), -(extents[2] + padding), 1});
    crop.setTranslation(xyz);
    crop.setRotation(rpy);
    crop.filter(result.second);

    std::vector<int> pts_to_remove;
    if (!result.second.empty()) {
        // query box with padding is always a superset of query box without padding
        crop.setIndices(boost_fake_shared(result.second));
        crop.setMax({extents[0], extents[1], extents[2], 1});
        crop.setMin({-extents[0], -extents[1], -extents[2], 1});
        crop.filter(pts_to_remove);
    }

    ROS_DEBUG_STREAM("Result size " << result.second.size() << "/" << result.first.size() << ", " << pts_to_remove.size());


    return std::make_tuple(result, pts_to_remove);
}

void CollectPoints::surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents,
                                    const std::function<void(uint32_t)> callback) {
    // Easy optimization: if no surfaces exist, don't call the callback at all
    if (highest_label_ < 0) {
        ROS_DEBUG_STREAM("Returning no surfaces because there are no surfaces");
        return;
    }

    // First limit to within the radius
    auto search_pt = surface_utils::pointFromVector<LabeledPoint>(center.translation());
    std::vector<int> indices;
    std::vector<float> sqr_distances_unused;
    // The largest diagonal of the bounding box happens to be equal to the l2 norm
    int n_in_radius = surface_points_.radiusSearch(search_pt, extents.norm(), indices, sqr_distances_unused);

    if (n_in_radius == 0) {
        return;
    }

    const auto pts = surface_points_.getInputCloud()->points;
    const auto tf = center.inverse();

    // Make sure each point is unique and within the transform
    std::vector<bool> seen_labels(static_cast<std::size_t>(highest_label_) + 1, false);
    for (auto &i : indices) {
        if (seen_labels[pts[i].label]) continue;

        LabeledPoint pt = pcl::transformPoint(pts[i], tf);
        if (std::abs(pt.x) <= extents[0] && std::abs(pt.y) <= extents[1] && std::abs(pt.z) <= extents[2]) {
            seen_labels[pts[i].label] = true;
            callback(pt.label);
        }
    }
}

void CollectPoints::add_surface(const CollectPoints::PointCloud &points, uint32_t label) {
    highest_label_ = std::max<int>(highest_label_, label);

    for (auto &pt : points.points) {
        LabeledPoint lpt;
        lpt.x = pt.x;
        lpt.y = pt.y;
        lpt.z = pt.z;
        lpt.label = label;
        surface_points_.addPointToCloud(lpt, surface_points_cloud_);
    }
}

void CollectPoints::remove_pending_points_near_surfaces() {
    PointCloud pending_pts;
    pending_points_.getVoxelCentroids(pending_pts.points);
    for (const auto &point : pending_pts) {
        if (inside_any_surface(point)) {
            pending_points_.deleteVoxelAtPoint(point);
        }
    }
}

void CollectPoints::remove_surface(uint32_t label) {
    // Find and remove all the points with label `label`
    // Note: don't just remove voxels, because that may delete more than it should
    auto n_removed = 0;
    for (auto lit = surface_points_.leaf_begin(); lit != surface_points_.leaf_end(); ++lit) {
        auto &leaf_indices = lit.getLeafContainer().getPointIndicesVector();
        auto remove_posn = std::remove_if(leaf_indices.begin(), leaf_indices.end(), [=](const int &idx) {
            return surface_points_cloud_->points[idx].label == label;
        });
        n_removed += std::distance(remove_posn, leaf_indices.end());
        leaf_indices.erase(remove_posn, leaf_indices.end());
    }
    ROS_DEBUG_STREAM("Removed " << n_removed << " points with label " << label << " from the surfaces octree");
}

void CollectPoints::remove_voxels_at_points(const PointCloud &points, std::vector<int> &indices) {
    if (indices.empty()) {
        return;
    }

    // Technically this does do what it says, but not in the way you would expect.
    // It also has a surprising requirement that points must contain every point currently in the octree
    auto new_pending_points_cloud = boost::make_shared<PendingPointsOctree::PointCloud>();
    std::sort(indices.begin(), indices.end());
    // Instead of having specific logic for running off the end of indices, add a past-the-end index that it will
    // never move past
    indices.push_back(static_cast<int>(points.size()));
    for (int point_i = 0, rm_i = 0; point_i < points.size(); point_i++) {
        assert(rm_i < indices.size() && "Ran off the end of indices");
        // indices[rm_i] is the next index to NOT copy
        if (point_i < indices[rm_i]) {
            // Then this index should be copied
            new_pending_points_cloud->push_back(points[point_i]);
        } else {
            // Then advance to the next index to NOT copy
            // Note that there may be duplicates in indices
            while (indices[++rm_i] <= point_i);
        }
        assert(point_i < indices[rm_i] && "Loop allowed the next index to NOT copy to fall behind the current index");
    }

    pending_points_cloud_ = new_pending_points_cloud;
    pending_points_ = PendingPointsOctree(pending_points_.getResolution());
    pending_points_.setInputCloud(new_pending_points_cloud);
    pending_points_.addPointsFromInputCloud();
}

size_t CollectPoints::num_pending_points() {
    // The VoxelCentroid octree returns one point for each extant leaf
    return pending_points_.getLeafCount();
}

void CollectPoints::update_surfaces(LabeledCloud::Ptr cloud, const int highest_label) {
    surface_points_cloud_ = std::move(cloud);
    surface_points_ = SurfacePointsOctree(surface_points_.getResolution());
    surface_points_.setInputCloud(surface_points_cloud_);
    surface_points_.addPointsFromInputCloud();
    highest_label_ = highest_label;
}

CollectPoints::LabeledCloud::Ptr CollectPoints::get_surface_points() {
    return surface_points_cloud_;
}
