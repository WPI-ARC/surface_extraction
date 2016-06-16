//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACEMANAGER_H
#define SURFACE_MANAGER_SURFACEMANAGER_H

// std and Boost
#include <future>

// ROS
#include <nodelet/nodelet.h>
#include <message_filters/subscriber.h>
#include <pcl_ros/point_cloud.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/mls.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/region_growing.h>

// Local
#include <surface_filters/ConcurrentPointGrouper.h>
#include <surface_msgs/SurfaceDetection.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>

namespace surface_manager {
    using namespace surface_msgs;

    class IncrementalSurfaceManager : public nodelet::Nodelet {
        using PointIn = pcl::PointXYZ;
        using PointCloudIn = pcl::PointCloud<PointIn>;
        using PointNormal = pcl::PointNormal;
        using PointCloudNormal = pcl::PointCloud<PointNormal>;

        virtual void onInit();

    public:
        virtual ~IncrementalSurfaceManager() {}

        void synchronized_input_callback(PointCloudIn::ConstPtr &input) const;

        auto downsample(PointCloudIn::ConstPtr &input) const -> PointCloudIn::Ptr&;
        auto changed(PointCloudIn::ConstPtr &input) const -> pcl::PointIndicesPtr&;
        auto crop(PointCloudIn::ConstPtr &input, SurfaceDetectionRequest &req) const -> std::pair<PointCloudIn::Ptr, PointCloudIn::Ptr>;
//        auto mls(PointCloudIn::ConstPtr &input) const -> PointCloudNormal::Ptr&;
//        auto region_growing(PointCloudIn::ConstPtr &input, PointCloudNormal::ConstPtr &normals) const -> std::vector<pcl::PointIndices>;
//        auto ransac(PointCloudIn::ConstPtr &input, pcl::PointIndices &cluster) const -> std::vector<std::pair<pcl::PointIndices, pcl::ModelCoefficients>>;

        bool get_surfaces(surface_msgs::SurfaceDetectionRequest &req, surface_msgs::SurfaceDetectionResponse &resp);

    private:


        ros::Subscriber sub_input_;
        float resolution_ = 0.02;
        mutable surface_filters::ConcurrentPointGrouper<PointIn> grouper_;
        mutable pcl::octree::OctreePointCloudChangeDetector<PointIn> change_detect_;
        ros::ServiceServer service_;
    };
}


#endif //SURFACE_MANAGER_SURFACEMANAGER_H
