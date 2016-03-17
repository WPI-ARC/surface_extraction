//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACEMANAGER_H
#define SURFACE_MANAGER_SURFACEMANAGER_H

#include <nodelet/nodelet.h>

#include <pcl/point_types.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/PointIndices.h>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>
#include <pcl/ModelCoefficients.h>
#include <surfaces/pcl_shim/ModelCoefficients_Serialization.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Mesh.h>
#include <surfaces/SurfaceMeshStamped.hpp>
#include <surfaces/SurfaceStamped.hpp>
#include <surfaces/SurfaceMeshes.hpp>
#include <surfaces/Surfaces.hpp>
#include <dynamic_reconfigure/server.h>
#include <surface_manager/SurfaceManagerConfig.h>
#include <pcl/common/time.h>

namespace vis = visualization_msgs;

namespace surface_manager {

    class SurfaceManager : public nodelet::Nodelet {

        typedef pcl::PointXYZ PointIn;
        typedef pcl::PointXYZRGB PointOut;
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef pcl::PointCloud<PointOut> PointCloudOut;


        typedef pcl_msgs::PolygonMesh PolygonMesh;
        typedef pcl_msgs::ModelCoefficients ModelCoefficients;

        typedef surfaces::Surface<PointIn> Surface;
        typedef surfaces::SurfaceStamped<PointIn> SurfaceStamped;
        typedef surfaces::Surfaces<PointIn> Surfaces;

        typedef surfaces::SurfaceMesh SurfaceMesh;
        typedef surfaces::SurfaceMeshStamped SurfaceMeshStamped;
        typedef surfaces::SurfaceMeshes SurfaceMeshes;

        typedef std::pair<Surface, SurfaceMesh> SurfaceMeshPair;

        template<typename ...S>
        using ExactTimeSynchronizer = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<S...> >;

        typedef ExactTimeSynchronizer<SurfaceStamped, SurfaceMeshStamped> SurfaceSynchronizer;

        virtual void onInit();

    private:
        ros::Publisher surfaces_pub_;
        ros::Publisher surface_meshes_pub_;
        ros::Publisher output_pub_;
        ros::Publisher perimeter_pub_;
        ros::Publisher visualization_pub_;

        ros::Timer publish_timer_;

        message_filters::Subscriber<SurfaceStamped>      new_surface_sub_;
        message_filters::Subscriber<SurfaceMeshStamped>  new_surface_mesh_sub_;
        message_filters::Subscriber<SurfaceStamped>      updated_surface_sub_;
        message_filters::Subscriber<SurfaceMeshStamped>  updated_surface_mesh_sub_;

        boost::shared_ptr<SurfaceSynchronizer> new_surface_synchronizer_;
        boost::shared_ptr<SurfaceSynchronizer> updated_surface_synchronizer_;

        boost::shared_ptr<dynamic_reconfigure::Server<SurfaceManagerConfig> > srv_;

        struct surface_lower_bound_comparator {
            bool operator()(const SurfaceMeshPair& a, const Surface& b) const {
                return (a.first.concave_hull.cloud.height * a.first.concave_hull.cloud.width) <
                        (b.concave_hull.cloud.height * b.concave_hull.cloud.width);
            }
        };

        std::vector<SurfaceMeshPair> surfaces_;

        int max_queue_size_;

        float publish_interval_;

        std::string target_frame_;

        unsigned int next_surface_id_ = 0;

        void add_surface(const SurfaceStamped::ConstPtr surface, const SurfaceMeshStamped::ConstPtr mesh);

        void replace_surface(const SurfaceStamped::ConstPtr surface, const SurfaceMeshStamped::ConstPtr mesh);

        void publish(const ros::TimerEvent &event) const;

        void publish_surfaces_mesh_pairs() const;
        void publish_inliers() const;
        void publish_perimeter_points() const;
        void publish_perimeter_lines() const;
        void publish_mesh_triangles() const;

        void config_callback(SurfaceManagerConfig &config, uint32_t level);

    };
}


#endif //SURFACE_MANAGER_SURFACEMANAGER_H
