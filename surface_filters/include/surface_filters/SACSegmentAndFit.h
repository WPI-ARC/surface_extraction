/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SAC_SEGMENT_AND_FIT_H
#define SAC_SEGMENT_AND_FIT_H

#include <limits.h>
#include <pcl_ros/pcl_nodelet.h>

// PCL includes
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/project_inliers.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/SACConfig.h>

#include <surfaces/PointClusters.hpp>
#include <surfaces/PointClusters_Serialization.hpp>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b RegionGrowingSegmentation represents a nodelet using the RegionGrowingSegmentation implementation.
    * \author Will Pryor
    */
    class SACSegmentAndFit : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn; // It actually will have RGB info too, but pcl_ros freaks out if you tell it that
        typedef pcl::PointNormal NormalIn;
        typedef pcl::PointXYZRGB PointOut;

        // Point Cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef pcl::PointCloud<NormalIn> NormalCloudIn;
        typedef pcl::PointCloud<PointOut> PointCloudOut;

        typedef pcl::PointIndices PointIndices;

        typedef surfaces::PointClusters PointClusters;

        // Message synchronizer types
        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> >;


    protected:
        /** \brief Set the model type */
        int model_type_ = 0;

        /** \brief Set the SAC method */
        int method_type_ = 0;

        /** \brief Set the threshold for distance to the model */
        double dist_threshold_ = 0;

        /** \brief Set the maximum number of iterations before giving up */
        int max_iterations_ = 50;

        /** \brief Set the probability of choosing at least one sample free from outliers */
        double probability_ = 0.99;

        /** \brief Set to true if a coefficient refinement is required */
        bool optimize_coefficients_ = true;

        /** \brief Set the minimum allowable radius for the model */
        double radius_min_ = INT_MIN;

        /** \brief Set the maximum allowable radius for the model */
        double radius_max_ = INT_MAX;

        /** \brief Set the maximum allowed difference between the model normal and the given axis in radians */
        double epsilon_angle_ = 0;

        /** \brief Set to true to use normals */
        bool use_normals_ = false;

        /** \brief Minimum number of points in a cluster */
        unsigned int min_points_ = 50;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<SACConfig> > srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(SACConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Input point cloud callback.
          * \param cloud the pointer to the input point cloud
          * \param indices the pointer to the input point cloud indices
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const NormalCloudIn::ConstPtr &normals,
                                         const PointClusters::ConstPtr &input_clusters);


    private:
        /** \brief The PCL implementation used. */
        pcl::SACSegmentation<PointIn> impl_;

        /** \brief The concave hull implementation, used for visualization only */
        pcl::ProjectInliers<PointIn> project_;

        /** \brief The clusters PointCloud publisher. */
        ros::Publisher pub_planes_;
        ros::Publisher pub_inliers_;

        /** \brief The message filter subscriber for PointCloud2. */
        // NOTE these have to come before the synchronizer members or you get a mutex lock error on destruction
        message_filters::Subscriber<NormalCloudIn> sub_normals_filter_;
        message_filters::Subscriber<PointClusters> sub_clusters_filter_;

        /** \brief Synchronized input, and indices.*/
        boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn, PointClusters> > sync_input_normals_clusters_e_;
        boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn, PointClusters> > sync_input_normals_clusters_a_;
        boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, PointClusters> > sync_input_clusters_e_;
        boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, PointClusters> > sync_input_clusters_a_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif // RANSAC_SEGMENT_AND_FIT_H
