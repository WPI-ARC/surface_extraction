//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/FreePointAccumulation.h>
#include <pcl/filters/extract_indices.h>

#define LOG(...) NODELET_DEBUG_STREAM("[" << getName().c_str() << ":" << __func__ << "] " << __VA_ARGS__)

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

    sub_found_inliers_ = pnh_->subscribe<PointIndices>("found_inliers", max_queue_size_,
                                                       bind(&FreePointAccumulation::found_inliers_callback, this, _1));

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

    inliers_filter_.setDistance(perpendicular_dist_threshold_);
    inliers_filter_.subscribe(*pnh_, "surfaces", max_queue_size_);

    NODELET_DEBUG("[%s::onInit] FreePointAccumulation Nodelet successfully created with connections:\n"
                  " - [subscriber] unindexed_input    : %s\n"
                  " - [subscriber] indexed_input      : %s\n"
                  " - [subscriber] indices            : %s\n"
                  " - [subscriber] surfaces           : %s\n"
                  " - [publisher]  accumulated_points : %s\n",
                  getName().c_str(), getMTPrivateNodeHandle().resolveName("unindexed_input").c_str(),
                  getMTPrivateNodeHandle().resolveName("indexed_input").c_str(),
                  getMTPrivateNodeHandle().resolveName("indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("surfaces").c_str(),
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

    NODELET_DEBUG_STREAM_THROTTLE(5, "[" << getName().c_str() << ":input_callback] "
                                         << "FreePointAccumulation has " << grouper_.size() << " pending points");

    do_publish();
}

void surface_filters::FreePointAccumulation::found_inliers_callback(const pcl::PointIndices::ConstPtr &indices) {
    auto cloud_it = processing_clouds_.find(indices->header);

    assert(cloud_it != processing_clouds_.end());

    std::vector<int> outlier_indices;

    pcl::ExtractIndices<PointIn> extract;
    extract.setInputCloud(cloud_it->second);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(outlier_indices);

    NODELET_INFO_STREAM("======>{\"event\": \"scan_processed\", \"value\": " << ros::Time::now() << ", \"cloud_stamp\": " << indices->header.stamp << "}");


//    LOG("Processing finished: had " << cloud_it->second->size() << " points, matched " << indices->indices.size()
//                                    << ", " << outlier_indices.size() << " remaining");

    grouper_.add(cloud_it->second, outlier_indices);

    NODELET_INFO_STREAM("======>{\"event\": \"accumulated_points\", \"value\": " << grouper_.size() << ", \"cloud_stamp\": " << indices->header.stamp << "}");

    processing_clouds_.erase(cloud_it);
}

void surface_filters::FreePointAccumulation::do_publish() {
    // Blocks until it's time to run OR returns false if another call to do_publish is taking care of it
    if (!publish_throttle_.runNow()) return;

    //! TEMPORARY: Only run one detection at a time
    if (processing_clouds_.size() > 1) return;

    std::vector<int> radius_filtered;

    auto points = grouper_.finish_group_if([&](const PointCloudIn::ConstPtr &accumulated_points) {
        // First, remove NaN (No idea where it came from but it ruins the further steps in the pipeline)

        // Simultaneously filter out the outliers and evaluate whether there are enough points to send to the pipeline
        radius_outlier_.setInputCloud(accumulated_points);
        auto indices = inliers_filter_.getFilteredIndices(accumulated_points);
        if (!indices) NODELET_WARN_STREAM("Inliers filter doesn't have any surfaces");
//        else LOG("Inliers filtered from " << accumulated_points->size() << " to " << indices->size() << " ("
//                 << (accumulated_points->size() - indices->size()) << " removed)");
        radius_outlier_.setIndices(indices);
        radius_outlier_.filter(radius_filtered);

        // Finish the group if there are enough points
        return radius_filtered.size() > 50; // TODO: Dynamic reconfigure for the parameter
    });

    if (!points) return;

    // Save and publish the accumulated inliers
    // Technically more efficient to publish the cloud and the list of points
    auto accumulated_inliers = boost::make_shared<PointCloudIn>(*points, radius_filtered);
    // This is sometimes NaN and filtering out the NaN doesn't work so just mark it as "may contain NaN"
    accumulated_inliers->is_dense = false;
    processing_clouds_[points->header] = accumulated_inliers;
    pub_accumulated_points_.publish(accumulated_inliers);

    LOG("Published points to new surface detection (" << processing_clouds_.size() << " clouds pending)");

    // Put the outliers back in the grouper for the next run
    // Note that this (correctly) doesn't re-add the points that were removed because they were surface inliers
    grouper_.add(points, *radius_outlier_.getRemovedIndices());
}

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
