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

using surface_utils::toLabeledPoint;

CollectPoints::CollectPoints(double discretization, double perpendicular_dist)
    : perpendicular_dist_(perpendicular_dist), surface_points_(discretization), pending_points_(discretization) {
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
    pending_points_.getVoxelCentroids(cloud.points);
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

    pcl::CropBox<Point> crop;

    crop.setInputCloud(boost::shared_ptr<PointCloud>(&result.first, null_deleter()));
    crop.setMax({extents[0], extents[1], extents[2], 1});
    crop.setMin({-extents[0], -extents[1], -extents[2], 1});
    crop.setTranslation(xyz);
    crop.setRotation(rpy);
    crop.filter(result.second.indices);

    return result;
}