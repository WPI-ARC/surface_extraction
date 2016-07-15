//
// Created by will on 6/21/16.
//

#ifndef PROJECT_COLLECTPOINTS_H
#define PROJECT_COLLECTPOINTS_H

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <tf/transform_listener.h>
#include <pcl/octree/octree_impl.h>

namespace pcl {
    class PointIndices;
}

#include <pcl/octree/impl/octree_base.hpp>

class CollectPoints {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::PointXYZL LabeledPoint;
    typedef pcl::PointCloud<LabeledPoint> LabeledCloud;

    typedef pcl::octree::OctreePointCloudSearch<LabeledPoint> SurfacePointsOctree;
    typedef pcl::octree::OctreePointCloudVoxelCentroid<Point> PendingPointsOctree;

    typedef std::pair<PointCloud, std::vector<int>> CloudIndexPair;

public:
    CollectPoints(double discretization, double perpendicular_dist, double point_inside_threshold, std::string target_frame, std::string camera_frame);

    void add_points(const PointCloud::ConstPtr &points);

    PointCloud get_pending_points();

    bool inside_any_surface(const Point &point) const;

    std::tuple<CloudIndexPair, std::vector<int>> pending_points_within(const Eigen::Affine3f &center,
                                                                       const Eigen::Vector3f &extents, float padding);

    void surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents, const std::function<void(uint32_t)> callback);

    void add_surface(const PointCloud &points, uint32_t label);

    void remove_surface(uint32_t label);

    void remove_voxels_at_points(const PointCloud &points, std::vector<int> &indices);

    void remove_pending_points_near_surfaces();

    size_t num_pending_points();

    void update_surfaces(LabeledCloud::Ptr cloud);

    LabeledCloud get_surface_points();

protected:
    double perpendicular_dist_;
    double point_inside_threshold_;
    std::string target_frame_;

    std::string camera_frame_;

    tf::TransformListener tf_listener_;
    SurfacePointsOctree surface_points_;
    PendingPointsOctree pending_points_;
    SurfacePointsOctree::PointCloud::Ptr surface_points_cloud_;
    PendingPointsOctree::PointCloud::Ptr pending_points_cloud_;
    int highest_label_ = -1;

};

#endif // PROJECT_COLLECTPOINTS_H
