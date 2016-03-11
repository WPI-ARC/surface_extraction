//
// Created by will on 2/15/16.
//

#ifndef SURFACE_MANAGER_SURFACEMANAGER_H
#define SURFACE_MANAGER_SURFACEMANAGER_H

#include <boost/thread/mutex.hpp>
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
#include <surfaces/PointIndices_Serialization.hpp>
#include <pcl/ModelCoefficients.h>
#include <surfaces/ModelCoefficients_Serialization.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//#include <surfaces/Polygons.hpp>
//#include <surfaces/Polygons_Serialization.hpp>
//#include <surfaces/Vertices_Serialization.hpp>
//#include <surfaces/Surface.hpp>
//#include <surfaces/Surfaces.hpp>
//#include <surfaces/Surfaces_Serialization.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <surface_msgs/Surface.h>
#include <surface_msgs/Surfaces.h>
#include <surface_msgs/SurfaceMesh.h>
#include <surface_msgs/SurfaceMeshes.h>
#include <surface_msgs/SurfaceCloud.h>
#include <surface_msgs/SurfaceClouds.h>
#include <surface_msgs/SurfaceStamped.h>
#include <pcl/surface/ear_clipping.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Mesh.h>

namespace vis = visualization_msgs;

namespace surface_manager {
    class SurfaceManager : public nodelet::Nodelet {

//        typedef pcl::PointXYZ PointIn;
        typedef sensor_msgs::PointCloud2 PointCloudIn;


        typedef pcl_msgs::PolygonMesh PolygonMesh;
        typedef pcl_msgs::ModelCoefficients ModelCoefficients;

        typedef surface_msgs::Surface Surface;
        typedef surface_msgs::SurfaceStamped SurfaceStamped;
        typedef surface_msgs::Surfaces Surfaces;

//        typedef surface_msgs::SurfaceCloud SurfaceCloud;
//        typedef surface_msgs::SurfaceClouds SurfaceClouds;

        typedef surface_msgs::SurfaceMesh SurfaceMesh;
        typedef surface_msgs::SurfaceMeshes SurfaceMeshes;

        template <bool IsManifoldT>
        struct MeshTraits
        {
            typedef int                                          VertexData;
            typedef pcl::geometry::NoData                        HalfEdgeData;
            typedef pcl::geometry::NoData                        EdgeData;
            typedef pcl::geometry::NoData                        FaceData;
            typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
        };

        template<typename ...S>
        using ExactTimeSynchronizer = message_filters::Synchronizer<message_filters::sync_policies::ExactTime<S...> >;

        typedef ExactTimeSynchronizer<PointCloudIn, PolygonMesh, ModelCoefficients> NewSurfaceSynchronizer;

        virtual void onInit();

    private:
        ros::Publisher surfaces_pub_;
//        ros::Publisher surface_clouds_pub_;
        ros::Publisher surface_meshes_pub_;
        ros::Publisher output_pub_;
        ros::Publisher perimeter_pub_;
        ros::Publisher visualization_pub_;

        ros::Timer publish_timer_;

        message_filters::Subscriber<PointCloudIn>  new_surface_inliers_sub_;
        message_filters::Subscriber<PolygonMesh> new_surface_convex_hull_sub_;
        message_filters::Subscriber<ModelCoefficients>  new_surface_plane_sub_;

        ros::Subscriber replace_surface_sub_;

        boost::shared_ptr<NewSurfaceSynchronizer> new_surface_synchronizer_;


        std::vector<Surface> surfaces;
//        std::vector<SurfaceCloud> surface_clouds;
        std::vector<SurfaceMesh> surface_meshes;

        void add_surface_synchronized(const PointCloudIn::ConstPtr inliers,
                                      const PolygonMesh::ConstPtr concave_hull,
                                      const ModelCoefficients::ConstPtr model);

        void replace_surface(const SurfaceStamped::ConstPtr new_surface);

        void publish(const ros::TimerEvent &event);

        shape_msgs::Mesh make_3d_mesh(PolygonMesh hull, Eigen::Affine3f tf);

    };
}


#endif //SURFACE_MANAGER_SURFACEMANAGER_H
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudOut;