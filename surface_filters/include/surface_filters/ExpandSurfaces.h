//
// Created by will on 2/17/16.
//

#ifndef SURFACE_FILTERS_EXPANDSURFACES_H
#define SURFACE_FILTERS_EXPANDSURFACES_H
#include <pcl_ros/pcl_nodelet.h>

#include <mutex>

// PCL includes
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/filters/crop_box.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

//#include <surfaces/Surfaces.hpp>
//#include <surfaces/Surfaces_Serialization.hpp>
#include <message_filters/cache.h>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>
#include <surface_msgs/Surface.h>
#include <surface_msgs/Surfaces.h>
#include <surface_msgs/SurfaceClouds.h>
#include <surface_msgs/SurfaceStamped.h>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b ChangeDetection subscribes to a stream of point clouds of the same scene and outputs the indices of the new points.
        * \author Will Pryor
        */
    class ExpandSurfaces : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn;

        // Point Cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef PointCloudIn PointCloudOut;

        typedef pcl::PointIndices PointIndices;

//        typedef surfaces::Surface<PointIn> Surface;
//        typedef surfaces::Surfaces<PointIn> Surfaces;
        typedef surface_msgs::Surface Surface;
        typedef surface_msgs::SurfaceStamped SurfaceStamped;
        typedef surface_msgs::Surfaces Surfaces;
        typedef surface_msgs::SurfaceClouds SurfaceClouds;

        typedef message_filters::sync_policies::ExactTime<PointCloud, pcl_msgs::PointIndices> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<PointCloud, pcl_msgs::PointIndices> ApproxPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactTimeSynchronizer;
        typedef message_filters::Synchronizer<ApproxPolicy> ApproxTimeSynchronizer;


    protected:
        /** \brief Discretization resolution */
        double perpendicular_dist_threshold_ = 0.025;
        double parallel_dist_threshold_ = 0.1;
        double concave_hull_alpha_ = 0.2;

        /** \brief Pointer to a dynamic reconfigure service. */
//        boost::shared_ptr<dynamic_reconfigure::Server<ChangeDetectionConfig> > srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
//        void config_callback(ChangeDetectionConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Callback that does all the work.
          * \param cloud the pointer to the input point cloud
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const pcl_msgs::PointIndices::ConstPtr &indices);


    private:
        pcl::search::KdTree<PointIn> search_;

        pcl::CropBox<PointIn> crop_;

        pcl::ConcaveHull<PointIn> hull_;

        /** \brief The input PointCloud subscriber */
        ros::Subscriber sub_input_;

        /** \brief Synchronized input and indices (used when 'input' is not the only required topic) */
        boost::shared_ptr<ExactTimeSynchronizer> sync_input_indices_e_;
        boost::shared_ptr<ApproxTimeSynchronizer> sync_input_indices_a_;


        message_filters::Subscriber<Surfaces> sub_surfaces_;
        message_filters::Subscriber<SurfaceClouds> sub_surface_clouds_;

        boost::shared_ptr<message_filters::Cache<Surfaces> > surfaces_cache_;
        boost::shared_ptr<message_filters::Cache<SurfaceClouds> > surface_clouds_cache_;

        /** \brief The output publisher. */
        ros::Publisher pub_replace_surface_;

        /** \brief The output publisher. */
        ros::Publisher pub_remaining_indices_;

        std::mutex hull_mutex_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //SURFACE_FILTERS_EXPANDSURFACES_H
