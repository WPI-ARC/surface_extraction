//
// Created by will on 2/17/16.
//

#ifndef SURFACE_FILTERS_EXPANDSURFACES_H
#define SURFACE_FILTERS_EXPANDSURFACES_H

// Standard and Boost includes
#include <atomic>
#include <mutex>
#include <unordered_map>

// ROS includes
#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

// Surfaces and Local includes
#include <surfaces/utils.hpp>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>
#include "ConcurrentPointGrouper.h"
#include "FilterSurfaceInliers.h"
#include "Throttle.h"

namespace surface_filters {
namespace sync_policies = message_filters::sync_policies;

/** \brief @b ChangeDetection subscribes to a stream of point clouds of the same scene and outputs the indices of the
 * new points.
    * \author Will Pryor
    */
class FreePointAccumulation : public pcl_ros::PCLNodelet {
    // Point types
    typedef pcl::PointXYZ PointIn;

    // Point Cloud types
    typedef pcl::PointCloud<PointIn> PointCloudIn;
    typedef PointCloudIn PointCloudOut;

    typedef pcl::PointIndices PointIndices;

    typedef message_filters::sync_policies::ExactTime<PointCloud, PointIndices> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<PointCloud, PointIndices> ApproxPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactTimeSynchronizer;
    typedef message_filters::Synchronizer<ApproxPolicy> ApproxTimeSynchronizer;

public:
    virtual ~FreePointAccumulation() {
        sync_input_indices_e_.reset();
        sync_input_indices_a_.reset();
    }

protected:
    /** \brief Discretization resolution */
    double perpendicular_dist_threshold_ = 0.10; // 0.025;

    double parallel_dist_threshold_ = 0.05; // 0.1;

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
    void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud, const PointIndices::ConstPtr &indices);

    void found_inliers_callback(const PointIndices::ConstPtr &indices);

    void do_publish();


private:
    pcl::RadiusOutlierRemoval<PointIn> radius_outlier_;

    FilterSurfaceInliers<PointIn> inliers_filter_;

    // NOTE this redefines sub_indices_filter_ to be a pcl::PointIndices instead of pcl_msgs::PointIndices
    // If this is removed then very frustrating compile errors will result
    message_filters::Subscriber<PointIndices> sub_indices_filter_;

    // Need another subscriber to handle non-indexed input
    ros::Subscriber sub_input_noindices_;
    ros::Subscriber sub_found_inliers_;

    /** \brief Synchronized input and indices (used when 'input' is not the only required topic) */
    boost::shared_ptr<ExactTimeSynchronizer> sync_input_indices_e_;
    boost::shared_ptr<ApproxTimeSynchronizer> sync_input_indices_a_;

    /** \brief The output publisher. */
    ros::Publisher pub_accumulated_points_;

    ConcurrentPointGrouper<PointIn> grouper_;


    std::unordered_map<pcl::PCLHeader, PointCloudIn::Ptr, surfaces::PCLHeaderHashNoSeq> processing_clouds_;

    Throttle publish_throttle_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#endif // SURFACE_FILTERS_EXPANDSURFACES_H
