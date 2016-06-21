//
// Created by will on 6/20/16.
//

#ifndef PROJECT_SURFACEDETECTION_H
#define PROJECT_SURFACEDETECTION_H

// PCL Includes
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/box_clipper3D.h>

// Surfaces includes
#include <surface_types/Surface.hpp>
#include <surface_types/Surfaces.hpp>
#include <surface_types/SurfaceMesh.hpp>
#include <surface_types/SurfaceMeshes.hpp>

// Utils includes
#include <surface_utils/conversions.hpp>
#include <surface_utils/multi_future.hpp>

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

    SurfaceDetection(double discretization, double perpendicular_distance)
        : // Params
          discretization_resolution_(discretization),
          perpendicular_dist_(perpendicular_distance),
          // State
          surface_points_(discretization_resolution_), pending_points_(discretization_resolution_) {
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
        return SurfaceMeshListPair();
//        return pending_points_within(center, extents)
//                ->then<SurfaceMeshListPair>([](PointCloud cloud, std::function<void(SurfaceMeshListPair)> yield) { yield(SurfaceMeshListPair()); })
//                ->get_one();
//            .then(expand_surfaces)
//            .then(detect_new_surfaces)
//            .then(merge_surfaces)
//            .get_one();
    }

    CloudIndexPair pending_points_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        auto result = std::make_pair(get_pending_points(), pcl::PointIndices());
        pcl::BoxClipper3D<Point>(center * Eigen::Scaling(extents)).clipPointCloud3D(result.first, result.second.indices);

        return result;
    }

private:
    // Configuration
    double discretization_resolution_;
    double perpendicular_dist_;

    // State
    SurfacePointsOctree surface_points_;
    PendingPointsOctree pending_points_;
    PendingPointsOctree::PointCloud::Ptr pending_points_cloud_;
};
}
#endif // PROJECT_SURFACEDETECTION_H
