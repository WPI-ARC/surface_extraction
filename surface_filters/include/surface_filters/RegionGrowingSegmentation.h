/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
#define SURFACE_FILTERS_CONCAVE_HULL_H_

// System and Boost includes
#include <limits.h>
#include <mutex>

// ROS includes
#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl/segmentation/region_growing.h>
// TODO: Is this impl header required?
#include <pcl/segmentation/impl/region_growing.hpp>

// Surfaces and Local includes
#include <surface_filters/RGSConfig.h>
#include <surfaces/PointClusters.hpp>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>

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

    // Shadow the parent class' version of these, which is the 'wrong' type
    typedef pcl::PointIndices PointIndices;
    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    // Message synchronizer types
    template <typename... SubscribedTypes>
    using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...>>;
    template <typename... SubscribedTypes>
    using ApproximateTimeSynchronizer =
        message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...>>;

public:
    virtual ~RegionGrowingSegmentation() {
        sync_input_normals_indices_e_.reset();
        sync_input_normals_indices_a_.reset();
        sync_input_normals_e_.reset();
        sync_input_normals_a_.reset();
    }

protected:
    /** \brief A pointer to the spatial search object. */
    SpatialSearch::Ptr tree_;

    std::mutex setup_mutex_;

    /** \brief Parameter for the spatial locator tree. By convention, the values represent:
      * 0: ANN (Approximate Nearest Neighbor library) kd-tree
      * 1: FLANN (Fast Library for Approximate Nearest Neighbors) kd-tree
      * 2: Organized spatial dataset index
      */
    int spatial_locator_type_ = 0;

    /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid */
    int min_cluster_size_ = 1;

    /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid */
    int max_cluster_size_ = INT_MAX;

    /** \brief Smooth Mode (if true, tests normals against neighbors' normals; otherwise tests normals against the
     * starting point's normal) */
    bool smooth_mode_flag_ = true;

    /** \brief Curvature test (if this is set to False, then Residual test will be set to True)    */
    bool curvature_test_flag_ = true;

    /** \brief Residual test (if this is set to False, then Curvature test will be set to True) */
    bool residual_test_flag_ = false;

    /** \brief Smoothness threshold for angle between normals */
    double smoothness_threshold_ = (30.0 / 180.0 * 3.141592653589793);

    /** \brief Residual threshold */
    double residual_threshold_ = 0.05;

    /** \brief Curvature threshold */
    double curvature_threshold_ = 0.05;

    /** \brief Number of neighbors used for knn */
    int num_neighbors_ = 30;

    /** \brief Pointer to a dynamic reconfigure service. */
    boost::shared_ptr<dynamic_reconfigure::Server<RGSConfig>> srv_;

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

    pcl::RegionGrowing<PointIn, NormalIn> getLocalRGSObject();

private:
    /** \brief The PCL implementation used. */
    pcl::RegionGrowing<PointIn, NormalIn> impl_;

    /** \brief The clusters PointCloud publisher. */
    ros::Publisher pub_clusters_;

    /** \brief Synchronized input, and indices.*/
    boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices>> sync_input_normals_indices_e_;
    boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices>>
        sync_input_normals_indices_a_;
    boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn>> sync_input_normals_e_;
    boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn>> sync_input_normals_a_;

    /** \brief The message filter subscriber for NormalCloudIn. */
    message_filters::Subscriber<NormalCloudIn> sub_normals_filter_;

    message_filters::Subscriber<PointIndices> sub_indices_filter_; // Shadow the parent's

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif //#ifndef PCL_ROS_MOVING_LEAST_SQUARES_H_
