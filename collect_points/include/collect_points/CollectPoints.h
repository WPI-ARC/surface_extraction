//
// Created by will on 6/21/16.
//

#ifndef PROJECT_COLLECTPOINTS_H
#define PROJECT_COLLECTPOINTS_H

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>

namespace pcl {
    class PointIndices;
}

class CollectPoints {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::PointXYZL LabeledPoint;

    typedef pcl::octree::OctreePointCloudSearch<LabeledPoint> SurfacePointsOctree;
    typedef pcl::octree::OctreePointCloudVoxelCentroid<Point> PendingPointsOctree;

    typedef std::pair<PointCloud, pcl::PointIndices> CloudIndexPair;

public:
    CollectPoints(double discretization, double perpendicular_dist);

    void add_points(const PointCloud::ConstPtr &points);

    PointCloud get_pending_points();

    bool inside_any_surface(const Point &point) const;

    CloudIndexPair pending_points_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents);

protected:
    double perpendicular_dist_;

    SurfacePointsOctree surface_points_;
    PendingPointsOctree pending_points_;
    PendingPointsOctree::PointCloud::Ptr pending_points_cloud_;
};

#endif // PROJECT_COLLECTPOINTS_H
