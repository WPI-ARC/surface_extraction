/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
#define SURFACE_FILTERS_CONCAVE_HULL_H_

#include <pcl_ros/pcl_nodelet.h>

// PCL includes
#include <pcl/surface/concave_hull.h>

#include <surfaces/Polygons.hpp>
#include <surfaces/Polygons_Serialization.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <surface_filters/ConcaveHullConfig.h>

namespace surface_filters {
    namespace sync_policies = message_filters::sync_policies;

    /** \brief @b MovingLeastSquares represents a nodelet using the MovingLeastSquares implementation.
    * The type of the output is the same as the input, it only smooths the XYZ coordinates according to the parameters.
    * Normals are estimated at each point as well and published on a separate topic.
    */
    class ConcaveHull : public pcl_ros::PCLNodelet {
        // Point types
        typedef pcl::PointXYZ PointIn;
        typedef pcl::PointXYZ PointOut;

        // Point cloud types
        typedef pcl::PointCloud<PointIn> PointCloudIn;
        typedef pcl::PointCloud<PointOut> PointCloudOut;

        // Vertex types
        typedef surfaces::Polygons Polygons;

        // Message synchronizer types
        template<typename ...SubscribedTypes>
        using ExactTimeSynchronizer = message_filters::Synchronizer<sync_policies::ExactTime<SubscribedTypes...> >;
        template<typename ...SubscribedTypes>
        using ApproximateTimeSynchronizer = message_filters::Synchronizer<sync_policies::ApproximateTime<SubscribedTypes...> >;

    protected:
        /** \brief positive, non-zero value, defining the maximum length from a vertex to the facet center. */
        double alpha_ = 0.025;

        /** \brief Dimension of the hull, either 2 or 3. */
        int dimension_ = 2;

        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<ConcaveHullConfig>> srv_;

        /** \brief Dynamic reconfigure callback
          * \param config the config object
          * \param level the dynamic reconfigure level
          */
        void config_callback(ConcaveHullConfig &config, uint32_t level);

    private:
        /** \brief Nodelet initialization routine. */
        virtual void onInit();

        /** \brief Synchronized input callback.
          * \param cloud the pointer to the input point cloud
          * \param indices the pointer to the input point cloud indices
          */
        void synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                         const PointIndices::ConstPtr &indices);


    private:
        /** \brief The PCL implementation used. */
        pcl::ConcaveHull<PointIn> impl_;

        /** \brief The input PointCloud subscriber (used when 'input' is the only required topic) */
        ros::Subscriber sub_input_;

        /** \brief Synchronized input and indices (used when 'input' is not the only required topic) */
        boost::shared_ptr<ExactTimeSynchronizer<PointCloudIn, PointIndices> > sync_input_indices_e_;
        boost::shared_ptr<ApproximateTimeSynchronizer<PointCloudIn, PointIndices> > sync_input_indices_a_;

        /** \brief The output PointCloud (containing the normals) publisher. */
        ros::Publisher pub_concave_hull_;

        /** \brief Mutex for use with dynamic reconfigure */
//        boost::recursive_mutex mutex_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif  //#ifndef SURFACE_FILTERS_CONCAVE_HULL_H_
