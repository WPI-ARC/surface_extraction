//
// Created by will on 6/20/16.
//

#ifndef PROJECT_SURFACEDETECTION_H
#define PROJECT_SURFACEDETECTION_H

// std s/ Boost include
#include <future>

// PCL Includes
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/eigen.h>

// Surface types includes
#include <surface_types/Surface.hpp>
#include <surface_types/Surfaces.hpp>
#include <surface_types/SurfaceMesh.hpp>
#include <surface_types/SurfaceMeshes.hpp>

// Algorithm includes
#include <expand_surfaces/ExpandSurfaces.hpp>

// Utils includes
#include <arc_utilities/pretty_print.hpp>
#include <surface_utils/conversions.hpp>
#include <surface_utils/multi_future.hpp>
#include "ProgressListener.hpp"

using surface_utils::toLabeledPoint;

namespace surface_detection {
class SurfaceDetection {
public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::PointXYZL LabeledPoint;

    typedef pcl::octree::OctreePointCloudSearch<LabeledPoint> SurfacePointsOctree;
    typedef pcl::octree::OctreePointCloudVoxelCentroid<Point> PendingPointsOctree;

    typedef surface_types::Surface<Point> Surface;
    typedef surface_types::Surfaces<Point> Surfaces;
    typedef surface_types::SurfaceMesh SurfaceMesh;
    typedef surface_types::SurfaceMeshes SurfaceMeshes;

    typedef std::pair<Surface, SurfaceMesh> SurfaceMeshPair;
    typedef std::pair<Surfaces, SurfaceMeshes> SurfaceMeshListPair;

    typedef std::pair<PointCloud, pcl::PointIndices> CloudIndexPair;

    SurfaceDetection(double discretization, double perpendicular_distance, double parallel_distance)
        : // Params
          discretization_resolution_(discretization),
          perpendicular_dist_(perpendicular_distance), parallel_dist_(parallel_distance),
          // State
          surface_points_(discretization_resolution_), pending_points_(discretization_resolution_), surfaces_(),
          // Implementation
          expand_surfaces_(perpendicular_distance, parallel_distance) {
        pending_points_cloud_ = boost::make_shared<PointCloud>();
        pending_points_.setInputCloud(pending_points_cloud_);
    }

    void add_points(const PointCloud::ConstPtr points) {
        for (const auto &point : *points) {
            if (!inside_any_surface(point)) {
                pending_points_.addPointToCloud(point, pending_points_cloud_);
            }
        }
    }

    PointCloud get_pending_points() {
        PointCloud cloud;
        pending_points_.getVoxelCentroids(cloud.points);
        return cloud;
    }

    bool inside_any_surface(const Point &point) const {
        std::vector<int> neighbors;
        std::vector<float> distances;

        return surface_points_.radiusSearch(toLabeledPoint(point), perpendicular_dist_, neighbors, distances, 1) > 0;
    }

    SurfaceMeshListPair detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        return detect_surfaces_within(center, extents, ProgressListener());
    }

    SurfaceMeshListPair detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents,
                                               ProgressListener p) {
        std::promise<SurfaceMeshListPair> finished_promise;
        auto points_to_process = pending_points_within(center, extents);
        ROS_DEBUG_STREAM("Processing " << points_to_process.second.indices.size() << " points");
        p.pair("points_to_process", points_to_process);
        auto remaining_indices = expand_surfaces_.expand_surfaces(
            surfaces_, points_to_process, [&finished_promise](Surface partial_surface) {
                ROS_DEBUG_STREAM("Got an expanded version of surface " << partial_surface.id);
            });

        finished_promise.set_value({});

        return finished_promise.get_future().get();
    }

    CloudIndexPair pending_points_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        auto result = std::make_pair(get_pending_points(), pcl::PointIndices());
        Eigen::Vector3f xyz, rpy;
        pcl::getTranslationAndEulerAngles(center, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
        ROS_DEBUG_STREAM("translation: " << PrettyPrint::PrettyPrint(xyz)
                                         << ", rotation: " << PrettyPrint::PrettyPrint(rpy)
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

private:
    // Configuration
    double discretization_resolution_;
    double perpendicular_dist_;
    double parallel_dist_;

    // State
    SurfacePointsOctree surface_points_;
    PendingPointsOctree pending_points_;
    PendingPointsOctree::PointCloud::Ptr pending_points_cloud_;
    std::map<int, SurfaceMeshPair> surfaces_;

    // Implementation
    ExpandSurfaces<Point> expand_surfaces_;
};
}
#endif // PROJECT_SURFACEDETECTION_H
