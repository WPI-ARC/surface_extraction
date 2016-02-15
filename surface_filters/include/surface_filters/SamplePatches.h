/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SURFACESTOCIRCLES_H
#define SURFACESTOCIRCLES_H

#include <pcl_ros/pcl_nodelet.h>

// PCL includes
#include <pcl/search/search.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/SamplePatchesConfig.h>

// Surfaces container objects and associated ROS message serialization
#include <surfaces/PointClusters.hpp>
#include <surfaces/PointClusters_Serialization.hpp>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b SamplePatches converts a list of clusters to a list of circular patches.
    * \author Will Pryor
    */
    class SamplePatches : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn;

        // Point Cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;

        typedef pcl::PointIndices PointIndices;

        typedef pcl::search::Search<PointIn> SpatialSearch;

        typedef surfaces::PointClusters PointClusters;

        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> > >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> > >;

    protected:
        /** \brief A pointer to the spatial search object. */
        SpatialSearch::Ptr tree_;

        /** \brief Parameter for the spatial locator tree. By convention, the values represent:
          * 0: ANN (Approximate Nearest Neigbor library) kd-tree
          * 1: FLANN (Fast Library for Approximate Nearest Neighbors) kd-tree
          * 2: Organized spatial dataset index
          */
        int spatial_locator_type_;

        /** \brief Minimum radius of a patch */
        double min_radius_;

        /** \brief Maximum radius of a patch */
        double max_radius_;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<SamplePatchesConfig> > srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(SamplePatchesConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Callback that does all the work.
          * \param cloud the pointer to the input point cloud
          * \param clusters the pointer to the input clusters
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const PointClusters::ConstPtr clusters);


    private:
        /** \brief The message filter subscriber for PointClusters. */
        message_filters::Subscriber<PointClusters> sub_clusters_filter_;

        /** \brief The clusters PointCloud publisher. */
        ros::Publisher pub_disc_patches_;

        /** \brief Message synchronizers.*/
        ExactTimeSynchronizer<PointCloudIn, PointClusters> sync_input_clusters_e_;
        ApproximateTimeSynchronizer<PointCloudIn, PointClusters> sync_input_clusters_a;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif // SURFACESTOCIRCLES_H
