//
// Created by will on 2/17/16.
//

#ifndef SURFACE_FILTERS_EXPANDSURFACES_H
#define SURFACE_FILTERS_EXPANDSURFACES_H

#include <atomic>
#include <limits>
#include <mutex>

// PCL includes
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

//#include <surfaces/Surfaces.hpp>
//#include <surfaces/Surfaces_Serialization.hpp>
#include <message_filters/cache.h>
#include <surfaces/pcl_shim/PointIndices_Serialization.hpp>
#include <surfaces/Surface.hpp>
#include <surfaces/Surfaces.hpp>
#include <surfaces/Segment.hpp>
#include <surfaces/utils.hpp>
#include "ConcurrentPointGrouper.h"
#include "FilterSurfaceInliers.h"

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

        typedef surfaces::Surface<PointIn> Surface;
        typedef surfaces::Surfaces<PointIn> Surfaces;

        typedef surfaces::Segment<PointIn> Segment;

        typedef std::map<unsigned int, PointCloudIn::Ptr> HullCloudsMap;

        typedef message_filters::sync_policies::ExactTime<PointCloud, PointIndices> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<PointCloud, PointIndices> ApproxPolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactTimeSynchronizer;
        typedef message_filters::Synchronizer<ApproxPolicy> ApproxTimeSynchronizer;

    public:
        virtual ~ExpandSurfaces() {
            sync_input_indices_e_.reset();
            sync_input_indices_a_.reset();
        }


    protected:
        /** \brief Discretization resolution */
        double perpendicular_dist_threshold_ = 0.03; //0.025;

        double parallel_dist_threshold_ = 0.05; //0.1;

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
        void input_callback(const Surfaces::ConstPtr &surfaces_in,
                            const PointCloudIn::ConstPtr &cloud,
                            const PointIndices::ConstPtr &indices);

        void process(const Surfaces::ConstPtr &surfaces, const PointCloudIn::ConstPtr &cloud, const PointIndices::ConstPtr &indices);

        HullCloudsMap getHullCloudsMap(const Surfaces::ConstPtr &surfaces) const;

        pcl::IndicesPtr filterWithinRadiusConnected(const pcl::search::Search<PointIn> &search,
                                                    const PointCloudIn::Ptr &edge_points,
                                                    const pcl::IndicesPtr &removed_indices) const;

    private:
        pcl::CropHull<PointIn> crophull_;

        /** \brief The input PointCloud subscriber */
        ros::Subscriber sub_input_;

        /** \brief Synchronized input and indices (used when 'input' is not the only required topic) */
        boost::shared_ptr<ExactTimeSynchronizer> sync_input_indices_e_;
        boost::shared_ptr<ApproxTimeSynchronizer> sync_input_indices_a_;

        message_filters::Subscriber<Surfaces> sub_surfaces_;
        message_filters::Subscriber<PointIndices> sub_pcl_indices_filter_;
        boost::shared_ptr<message_filters::Cache<Surfaces> > surfaces_cache_;

        /** \brief The output publisher. */
        ros::Publisher pub_replace_surface_;

        /** \brief The output publisher. */
        ros::Publisher pub_remaining_indices_;
        ros::Publisher pub_filtered_indices_;
        ros::Publisher pub_removed_indices_;
        ros::Publisher pub_existing_inliers_;

        std::mutex hull_mutex_;

        ConcurrentPointGrouper<PointIn> grouper_;
        FilterSurfaceInliers<PointIn> filter_inliers_;

        std::atomic<decltype(pcl::PCLHeader().stamp)> latest_update_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //SURFACE_FILTERS_EXPANDSURFACES_H
