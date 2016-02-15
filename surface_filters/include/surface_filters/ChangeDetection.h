/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef CHANGEDETECTION_H
#define CHANGEDETECTION_H

#include <pcl_ros/pcl_nodelet.h>

// PCL includes
#include <pcl/octree/octree.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/ChangeDetectionConfig.h>

// Surfaces container objects and associated ROS message serialization
#include <surfaces/PointClusters.hpp>
#include <surfaces/PointClusters_Serialization.hpp>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b ChangeDetection subscribes to a stream of point clouds of the same scene and outputs the indices of the new points.
    * \author Will Pryor
    */
    class ChangeDetection : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn;
        typedef PointIn PointOut;

        // Point Cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef PointCloudIn PointCloudOut;

        typedef pcl::PointIndices PointIndices;

        typedef pcl::octree::OctreePointCloudChangeDetector<PointIn> ChangeDetector;

        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> > >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> > >;

    protected:
        /** \brief Discretization resolution */
        double resolution_ = 0.025;

        /** \brief Minimum number of points required  for a leaf to be output */
        int min_points_in_leaf_ = 0;

        /** \brief Percentage of points which must be new for the cloud to be considered a new scene */
        double new_scene_threshold_ = 0.2;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<ChangeDetectionConfig> > srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(ChangeDetectionConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Callback that does all the work.
          * \param cloud the pointer to the input point cloud
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud);


    private:
        /** \brief The PCL Point cloud change detector */
        ChangeDetector impl_ = ChangeDetector(resolution_);

        /** \brief The input PointCloud subscriber */
        ros::Subscriber sub_input_;

        /** \brief The indices publisher. */
        ros::Publisher pub_indices_;

        /** \brief The new scene publisher. */
        ros::Publisher pub_new_scene_;

        /** \brief The output publisher. */
        ros::Publisher pub_output_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif // CHANGEDETECTION_H
