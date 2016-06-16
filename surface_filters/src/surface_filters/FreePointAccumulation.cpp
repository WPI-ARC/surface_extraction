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

    publish_throttle_.setInterval(ros::Duration(0.5), ros::Duration(0.5));

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

    std::lock_guard<std::mutex> lock(publish_future_mutex_);
    if (!publish_future_.valid() || publish_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        NODELET_DEBUG_STREAM("Starting a publish thread");
        publish_future_ = std::async(std::launch::async, &FreePointAccumulation::do_publish, this);
    }
}

void surface_filters::FreePointAccumulation::found_inliers_callback(const pcl::PointIndices::ConstPtr &indices) {
    auto cloud = get_processing_cloud(indices->header);

    assert(cloud != nullptr && "The cloud associated with these indices was not in the map of stored clouds");

    std::vector<int> outlier_indices;

    pcl::ExtractIndices<PointIn> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(outlier_indices);

    grouper_.add(cloud, outlier_indices);
}

void surface_filters::FreePointAccumulation::do_publish() {
    // Blocks until it's time to run OR returns false if another call to do_publish is taking care of it
    if (!publish_throttle_.runNow()) return;

    std::lock_guard<std::mutex> processing_clouds_lock(processing_clouds_mutex_);
    //! TEMPORARY: Only run one detection at a time
    if (processing_clouds_.size() > 1) return;

    auto points = get_points_to_process();

    if (!points) return;
    NODELET_WARN_STREAM("Removed " << points->size() << " points for processing");

    // Save and publish the accumulated inliers
    // Technically more efficient to publish the cloud and the list of points
    // This is sometimes NaN and filtering out the NaN doesn't work so just mark it as "may contain NaN"
    points->is_dense = false;
    processing_clouds_[points->header] = points;
    pub_accumulated_points_.publish(points);

    LOG("Published points to new surface detection (" << processing_clouds_.size() << " clouds pending)");
}

auto surface_filters::FreePointAccumulation::get_points_to_process() -> PointCloudIn::Ptr {
    PointCloudIn::Ptr accumulated_points = this->grouper_.finish_group();

    pcl::RadiusOutlierRemoval<PointIn> radius_outlier_(true);
    pcl::ExtractIndices<PointIn> extract;

    auto radius_filtered = boost::make_shared<std::vector<int>>();
    auto cloud_to_process = boost::make_shared<PointCloudIn>();
    auto cloud_to_keep = boost::make_shared<PointCloudIn>();

    NODELET_INFO_STREAM("Starting with " << accumulated_points->size() << " points");

    // Get a list of indices excluding inliers to existing planes
    auto indices = this->inliers_filter_.getFilteredIndices(accumulated_points);
    if (!indices) NODELET_WARN_STREAM("Inliers filter doesn't have any surfaces");
    else NODELET_INFO_STREAM("Filtered to " << indices->size() << " points that aren't plane inliers");

    // Filter indices to only those which aren't radius outliers
    radius_outlier_.setRadiusSearch(0.25);
    radius_outlier_.setMinNeighborsInRadius(10);
    radius_outlier_.setInputCloud(accumulated_points);
    if (indices) radius_outlier_.setIndices(indices);
    radius_outlier_.filter(*radius_filtered);

    NODELET_INFO_STREAM("Filtered to " << radius_filtered->size() << " points that aren't radius outliers");

    // If there are fewer than 50 unfiltered points, don't process anything
    if (radius_filtered->size() < 50) {
        NODELET_INFO_STREAM("Too few points -- not processing");
        // Use cloud_to_process just to get a null pointer with the correct type
        cloud_to_keep = accumulated_points;
    }

    extract.setInputCloud(accumulated_points);

    // Use ExtractIndices to get the cloud_to_process
    extract.setIndices(radius_filtered);
    extract.filter(*cloud_to_process);

    assert(cloud_to_process->size() == radius_filtered->size() && "Incorrect cloud_to_process output from ExtractIndices");

    // Use ExtractIndices to get the cloud_to_keep
    extract.setIndices(radius_outlier_.getRemovedIndices());
    extract.filter(*cloud_to_keep);

    assert(cloud_to_keep->size() == radius_outlier_.getRemovedIndices()->size() && "Incorrect cloud_to_keep output from ExtractIndices");

    NODELET_INFO_STREAM("Discarding " << (accumulated_points->size() - cloud_to_process->size() - cloud_to_keep->size()) << " plane inliers");

    this->grouper_.add(cloud_to_keep);
    return cloud_to_process;
}

auto surface_filters::FreePointAccumulation::get_processing_cloud(const pcl::PCLHeader &header) -> PointCloudIn::Ptr {
    std::lock_guard<std::mutex> processing_clouds_lock(processing_clouds_mutex_);

    auto cloud_it = processing_clouds_.find(header);

    NODELET_DEBUG_STREAM(processing_clouds_.size() << " clouds stored, with headers:");
    for (const auto &cloud : processing_clouds_) {
        NODELET_DEBUG_STREAM("Frame: " << cloud.first.frame_id << ", seq: " << cloud.first.seq << ", stamp: " << cloud.first.stamp);
    }
    NODELET_DEBUG_STREAM("Looking for header:");
    NODELET_DEBUG_STREAM("Frame: " << header.frame_id << ", seq: " << header.seq << ", stamp: " << header.stamp);

    if (cloud_it == processing_clouds_.end()) {
        return nullptr;
    }

    PointCloudIn::Ptr cloud = cloud_it->second;
    processing_clouds_.erase(cloud_it);

    return cloud;
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
