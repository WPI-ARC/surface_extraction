//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACEMANAGER_H
#define SURFACE_MANAGER_SURFACEMANAGER_H

#include <boost/thread/mutex.hpp>
#include <nodelet/nodelet.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <pcl/PointIndices.h>
#include <surfaces/PointIndices_Serialization.hpp>
#include <pcl/ModelCoefficients.h>
#include <surfaces/ModelCoefficients_Serialization.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <surfaces/Polygons.hpp>
#include <surfaces/Polygons_Serialization.hpp>
#include <surfaces/Vertices_Serialization.hpp>
#include <surfaces/Surface.hpp>
#include <surfaces/Surfaces.hpp>
#include <surfaces/Surfaces_Serialization.hpp>

namespace surface_manager {
    class SurfaceManager : public nodelet::Nodelet {
        typedef pcl::PointXYZ PointIn;
        typedef pcl::PointCloud<PointIn> PointCloudIn;

        template<typename ...S>
        using ExactTimeSynchronizer = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<S...> >;

        typedef ExactTimeSynchronizer<PointCloudIn, surfaces::Polygons, pcl::ModelCoefficients> NewSurfaceSynchronizer;

        virtual void onInit();

    private:
        ros::Publisher surfaces_pub_;

        message_filters::Subscriber<PointCloudIn>  new_surface_inliers_sub_;
        message_filters::Subscriber<surfaces::Polygons> new_surface_convex_hull_sub_;
        message_filters::Subscriber<pcl::ModelCoefficients>  new_surface_plane_sub_;

        boost::shared_ptr<NewSurfaceSynchronizer> new_surface_synchronizer_;

        std::vector<surfaces::Surface<PointIn> > surfaces;
//        std::vector<surfa

        void add_surface_synchronized(pcl::PointCloud<pcl::PointXYZ>::ConstPtr inliers,
                                      surfaces::Polygons::ConstPtr convex_hull, pcl::ModelCoefficients::ConstPtr model);
    };
}


#endif //SURFACE_MANAGER_SURFACEMANAGER_H
