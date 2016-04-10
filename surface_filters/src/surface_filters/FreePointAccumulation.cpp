//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/FreePointAccumulation.h>

//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::FreePointAccumulation::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_accumulated_points_ = pnh_->advertise<PointCloudIn>("accumulated_points", max_queue_size_);

    // Enable the dynamic reconfigure service
    //    srv_ = boost::make_shared<dynamic_reconfigure::Server<ChangeDetectionConfig> >(*pnh_);
    //    srv_->setCallback(boost::bind(&ChangeDetection::config_callback, this, _1, _2));

    sub_input_noindices_ = pnh_->subscribe<PointCloudIn>(
        "unindexed_input", max_queue_size_,
        bind(&FreePointAccumulation::synchronized_input_callback, this, _1, PointIndices::Ptr()));

    sub_input_filter_.subscribe(*pnh_, "indexed_input", max_queue_size_);
    sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

    if (approximate_sync_) {
        sync_input_indices_a_ = boost::make_shared<ApproxTimeSynchronizer>(ApproxPolicy(max_queue_size_),
                                                                           sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(
            bind(&FreePointAccumulation::synchronized_input_callback, this, _1, _2));
    } else {
        sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer>(ExactPolicy(max_queue_size_),
                                                                          sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(
            bind(&FreePointAccumulation::synchronized_input_callback, this, _1, _2));
    }

    // TODO: Dynamic reconfigure for these params
    radius_outlier_ = pcl::RadiusOutlierRemoval<PointIn>(true); // Allow us to query for the removed points
    radius_outlier_.setMinNeighborsInRadius(10);
    radius_outlier_.setRadiusSearch(0.25);

    publish_throttle_.setInterval(ros::Duration(0.5));

    NODELET_DEBUG("[%s::onInit] FreePointAccumulation Nodelet successfully created with connections:\n"
                  " - [subscriber] unindexed_input    : %s\n"
                  " - [subscriber] indexed_input      : %s\n"
                  " - [subscriber] indices            : %s\n"
                  " - [publisher]  accumulated_points : %s\n",
                  getName().c_str(), getMTPrivateNodeHandle().resolveName("unindexed_input").c_str(),
                  getMTPrivateNodeHandle().resolveName("indexed_input").c_str(),
                  getMTPrivateNodeHandle().resolveName("indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("accumulated_points").c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::FreePointAccumulation::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud_in,
                                                                         const PointIndices::ConstPtr &indices_in) {
    // pcl::ScopeTime scopetime("Expand Surfaces");

    if (cloud_in->size() == 0 || (indices_in && indices_in->indices.size() == 0)) {
        return;
    }

    assert(cloud_in); // Assert non-null
    grouper_.add(cloud_in, indices_in);

    NODELET_DEBUG_STREAM_THROTTLE(5, "[" << getName().c_str() << ":synchronized_input_callback] "
                                         << "FreePointAccumulation has " << grouper_.size() << " pending points");

    do_publish();
}

void surface_filters::FreePointAccumulation::do_publish() {
    // Blocks until it's time to run OR returns false if another call to do_publish is taking care of it
    if (!publish_throttle_.runNow()) return;

    std::vector<int> inlier_indices;

    auto points = grouper_.finish_group_if([&](const PointCloudIn::ConstPtr &accumulated_points) {
        // Simultaneously filter out the outliers and evaluate whether there are enough points to send to the pipeline
        radius_outlier_.setInputCloud(accumulated_points);
        radius_outlier_.filter(inlier_indices);

        // Finish the group if there are enough points
        return inlier_indices.size() > 50; // TODO: Dynamic reconfigure for the parameter
    });

    if (!points) return;

    // Save and publish the accumulated inliers
    auto accumulated_inliers = boost::make_shared<PointCloudIn>(*points, inlier_indices);
    processing_clouds_[points->header] = accumulated_inliers;
    pub_accumulated_points_.publish(accumulated_inliers);

    // Put the outliers back in the grouper for the next run
    grouper_.add(points, *radius_outlier_.getRemovedIndices());
}

void surface_filters::FreePointAccumulation::do_publish(const ros::TimerEvent &) { do_publish(); }

//////////////////////////////////////////////////////////////////////////////////////////////
// void surface_filters::FreePointAccumulation::config_callback(ChangeDetectionConfig &config, uint32_t level) {
//    if (resolution_ != config.resolution) {
//        resolution_ = config.resolution;
//        hull_ = ChangeDetector(resolution_);
//        NODELET_DEBUG("[config_callback] Setting the resolution to: %f. Note that changing the resolution requires
//        restarting change detection", resolution_);
//    }
//
//    if (min_points_in_leaf_ != config.min_points_per_leaf) {
//        min_points_in_leaf_ = config.min_points_per_leaf;
//        NODELET_DEBUG("[config_callback] Setting the max radius to: %d.", min_points_in_leaf_);
//    }
//}

PLUGINLIB_EXPORT_CLASS(surface_filters::FreePointAccumulation, nodelet::Nodelet)
