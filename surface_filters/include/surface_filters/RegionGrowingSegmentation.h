/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef PCL_ROS_MOVING_LEAST_SQUARES_H_
#define PCL_ROS_MOVING_LEAST_SQUARES_H_

#include <pcl_ros/pcl_nodelet.h>

// PCL includes
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/search/pcl_search.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/RGSConfig.h>

#include <surfaces/PointClusters.h>
#include <surfaces/PointClusters_Serialization.h>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b RegionGrowingSegmentation represents a nodelet using the RegionGrowingSegmentation implementation.
    * \author Will Pryor
    */
    class RegionGrowingSegmentation : public pcl_ros::PCLNodelet {
        // Point Types
        typedef pcl::PointXYZ PointIn;
        typedef pcl::PointNormal NormalIn;
        typedef pcl::PointXYZRGB PointOut;

        // Point Cloud Types
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef pcl::PointCloud<NormalIn> NormalCloudIn;
        typedef pcl::PointCloud<PointOut> PointCloudOut;

        typedef surfaces::PointClusters PointClusters;

        typedef pcl::search::Search<PointIn> SpatialSearch;

        // Message synchronizer types
        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> >;

    protected:
        /** \brief A pointer to the spatial search object. */
        SpatialSearch::Ptr tree_;

        /** \brief Parameter for the spatial locator tree. By convention, the values represent:
          * 0: ANN (Approximate Nearest Neighbor library) kd-tree
          * 1: FLANN (Fast Library for Approximate Nearest Neighbors) kd-tree
          * 2: Organized spatial dataset index
          */
        int spatial_locator_type_;

        /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid */
        int min_cluster_size_;

        /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid */
        int max_cluster_size_;

        /** \brief Smooth Mode (if true, tests normals against neighbors' normals; otherwise tests normals against the starting point's normal) */
        bool smooth_mode_flag_;

        /** \brief Curvature test (if this is set to False, then Residual test will be set to True)    */
        bool curvature_test_flag_;

        /** \brief Residual test (if this is set to False, then Curvature test will be set to True) */
        bool residual_test_flag_;

        /** \brief Smoothness threshold for angle between normals */
        double smoothness_threshold_;

        /** \brief Residual threshold */
        double residual_threshold_;

        /** \brief Curvature threshold */
        double curvature_threshold_;

        /** \brief Number of neighbors used for knn */
        int num_neighbors_;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<RGSConfig> > srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(RGSConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Input point cloud callback.
          * \param cloud the pointer to the input point cloud
          * \param indices the pointer to the input point cloud indices
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const NormalCloudIn::ConstPtr &normals,
                                         const PointIndices::ConstPtr &indices);


    private:
        /** \brief The PCL implementation used. */
        pcl::RegionGrowing<PointIn, NormalIn> impl_;

        /** \brief The clusters PointCloud publisher. */
        ros::Publisher pub_clusters_;

        /** \brief Synchronized input, and indices.*/
        boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices> > sync_input_normals_indices_e_;
        boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices> > sync_input_normals_indices_a_;
        boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn> > sync_input_normals_e_;
        boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn> > sync_input_normals_a_;

        /** \brief The message filter subscriber for NormalCloudIn. */
        message_filters::Subscriber<NormalCloudIn> sub_normals_filter_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void setup_spatial_locator(int type);
    } __unused;
}
#endif  //#ifndef PCL_ROS_MOVING_LEAST_SQUARES_H_
