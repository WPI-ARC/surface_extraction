//
// Created by will on 6/21/16.
//

#include "collect_points/CollectPoints.h"

#include <ros/console.h>

// PCL Includes
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h>

// Utils
#include <arc_utilities/pretty_print.hpp>
#include <surface_utils/conversions.hpp>
#include <surface_utils/smart_ptr.hpp>
#include <pcl/common/time.h>
#include <eigen_conversions/eigen_msg.h>

using surface_utils::toLabeledPoint;

CollectPoints::CollectPoints(double discretization, double perpendicular_dist, std::string target_frame,
                             std::string camera_frame)
    : perpendicular_dist_(perpendicular_dist), target_frame_(target_frame), camera_frame_(camera_frame),
      silence_tf_warnings_until_(ros::Time::now() + ros::Duration(5)), tf_listener_(), surface_points_(discretization),
      pending_points_(discretization) {
    pending_points_cloud_ = boost::make_shared<PointCloud>();
    pending_points_.setInputCloud(pending_points_cloud_);
}

void CollectPoints::add_points(const PointCloud::ConstPtr &points) {
    for (const auto &point : *points) {
        if (!inside_any_surface(point)) {
            pending_points_.addPointToCloud(point, pending_points_cloud_);
        }
    }
}

CollectPoints::PointCloud CollectPoints::get_pending_points() {
    PointCloud cloud;
    // Assume the current camera transform is the viewpoint of all the pending points
    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(target_frame_, camera_frame_, ros::Time(0), transform);
    } catch (tf2::TransformException ex) {
        if (ros::Time::now() > silence_tf_warnings_until_) {
            ROS_WARN_STREAM("Failed to get camera viewpoint: " << ex.what());
        }
        return cloud; // Return an empty cloud
    }
    pending_points_.getVoxelCentroids(cloud.points);
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
    return cloud;
}

bool CollectPoints::inside_any_surface(const Point &point) const {
    std::vector<int> neighbors;
    std::vector<float> distances;

    return surface_points_.radiusSearch(toLabeledPoint(point), perpendicular_dist_, neighbors, distances, 1) > 0;
}

CollectPoints::CloudIndexPair CollectPoints::pending_points_within(const Eigen::Affine3f &center,
                                                                   const Eigen::Vector3f &extents) {
    pcl::ScopeTime st("CollectPoints::pending_points_within");
    auto result = std::make_pair(get_pending_points(), pcl::PointIndices());
    Eigen::Vector3f xyz, rpy;
    pcl::getTranslationAndEulerAngles(center, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
    ROS_DEBUG_STREAM("translation: " << PrettyPrint::PrettyPrint(xyz) << ", rotation: " << PrettyPrint::PrettyPrint(rpy)
                                     << ", size: " << PrettyPrint::PrettyPrint(extents));
    ROS_DEBUG_STREAM("Pending points have sensor origin <"
                     << result.first.sensor_origin_[0] << ", " << result.first.sensor_origin_[1] << ", "
                     << result.first.sensor_origin_[2] << ", " << result.first.sensor_origin_[3] << ">");

    pcl::CropBox<Point> crop;

    crop.setInputCloud(boost::shared_ptr<PointCloud>(&result.first, null_deleter()));
    crop.setMax({extents[0], extents[1], extents[2], 1});
    crop.setMin({-extents[0], -extents[1], -extents[2], 1});
    crop.setTranslation(xyz);
    crop.setRotation(rpy);
    crop.filter(result.second.indices);

    return result;
}