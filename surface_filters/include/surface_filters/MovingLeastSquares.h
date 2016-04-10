/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
#define SURFACE_FILTERS_CONCAVE_HULL_H_

// System and Boost includes
#include <mutex>

// ROS includes
#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl/surface/mls.h>

// Surfaces and Local includes
#include <surface_filters/MLSConfig.h>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>

namespace surface_filters {
namespace sync_policies = message_filters::sync_policies;

/** \brief @b MovingLeastSquares represents a nodelet using the MovingLeastSquares implementation.
* The type of the output is the same as the input, it only smooths the XYZ coordinates according to the parameters.
* Normals are estimated at each point as well and published on a separate topic.
*/
class MovingLeastSquaresNodelet : public pcl_ros::PCLNodelet {
    // Point types
    typedef pcl::PointXYZ PointIn;
    typedef pcl::PointXYZ PointOut;
    typedef pcl::PointNormal NormalOut;

    // Point cloud types
    typedef pcl::PointCloud<PointIn> PointCloudIn;
    typedef pcl::PointCloud<PointOut> PointCloudOut;
    typedef pcl::PointCloud<NormalOut> NormalCloudOut;

    // PCL object types
    typedef pcl::search::Search<PointIn> SpatialSearch;

    typedef pcl::PointIndices PointIndices; // Shadow superclass's typedef

    // Message synchronizer types
    template <typename... SubscribedTypes>
    using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...>>;
    template <typename... SubscribedTypes>
    using ApproximateTimeSynchronizer =
        message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...>>;

public:
    virtual ~MovingLeastSquaresNodelet() {
        sync_input_indices_a_.reset();
        sync_input_indices_e_.reset();
    }

protected:
    std::mutex setup_mutex_;

    /** \brief The nearest neighbors search radius for each point. */
    double search_radius_ = 0.02;

    /** \brief Whether to use a polynomial fit. */
    bool use_polynomial_fit_ = true;

    /** \brief The order of the polynomial to be fit. */
    int polynomial_order_ = 2;

    /** \brief How 'flat' should the neighbor weighting gaussian be (the smaller, the more local the fit). */
    double gaussian_parameter_ = 0.02;

    /** \brief Whether the node should output point normals */
    bool compute_normals_ = false;

    /** \brief Parameter for the spatial locator tree. By convention, the values represent:
      * 0: ANN (Approximate Nearest Neighbor library) kd-tree
      * 1: FLANN (Fast Library for Approximate Nearest Neighbors) kd-tree
      * 2: Organized spatial dataset index
      */
    int spatial_locator_type_ = 0;

    /** \brief Pointer to a dynamic reconfigure service. */
    boost::shared_ptr<dynamic_reconfigure::Server<MLSConfig>> srv_;

    /** \brief Dynamic reconfigure callback
      * \param config the config object
      * \param level the dynamic reconfigure level
      */
    void config_callback(MLSConfig &config, uint32_t level);

private:
    /** \brief Nodelet initialization routine. */
    virtual void onInit();

    /** \brief Synchronized input callback.
      * \param cloud the pointer to the input point cloud
      * \param indices the pointer to the input point cloud indices
      */
    void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const PointIndices::ConstPtr &indices);

    pcl::MovingLeastSquares<PointIn, NormalOut> getLocalMLSObject();

private:
    /** \brief The PCL implementation used. */
    pcl::MovingLeastSquares<PointIn, NormalOut> impl_;

    /** \brief The input PointCloud subscriber (used when 'input' is the only required topic) */
    ros::Subscriber sub_input_;

    /** \brief The message filter subscriber for PointCloud2. */
    // Shadow superclass's version, which is templated on an incompatible type
    message_filters::Subscriber<PointIndices> sub_indices_filter_;

    /** \brief Synchronized input and indices (used when 'input' is not the only required topic) */
    boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, PointIndices>> sync_input_indices_e_;
    boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, PointIndices>> sync_input_indices_a_;

    /** \brief The output PointCloud (containing the normals) publisher. */
    ros::Publisher pub_normals_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif //#ifndef PCL_ROS_MOVING_LEAST_SQUARES_H_
