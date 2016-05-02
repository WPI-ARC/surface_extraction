//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ExpandSurfaces.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>
//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::ExpandSurfaces::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_replace_surface_ = pnh_->advertise<Segment>("expanded_segments", max_queue_size_);
    pub_remaining_indices_ = pnh_->advertise<PointIndices>("remaining_indices", max_queue_size_);
    pub_filtered_indices_ = pnh_->advertise<PointIndices>("filtered_indices", max_queue_size_);
    pub_removed_indices_ = pnh_->advertise<PointIndices>("removed_indices", max_queue_size_);
    pub_existing_inliers_ = pnh_->advertise<PointCloudIn>("existing_inliers", max_queue_size_);

    //    if (!pnh_->getParam("resolution", resolution_)) {
    //        NODELET_ERROR("[%s::onInit] Need a 'resolution' parameter to be set before continuing!",
    //                      getName().c_str());
    //        return;
    //    }

    // Enable the dynamic reconfigure service
    //    srv_ = boost::make_shared<dynamic_reconfigure::Server<ChangeDetectionConfig> >(*pnh_);
    //    srv_->setCallback(boost::bind(&ChangeDetection::config_callback, this, _1, _2));

    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    sub_pcl_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

    pnh_->getParam("parallel_distance_threshold", parallel_dist_threshold_);
    pnh_->getParam("perpendicular_distance_threshold", perpendicular_dist_threshold_);


    if (approximate_sync_) {
        sync_input_indices_a_ = boost::make_shared<ApproxTimeSynchronizer>(ApproxPolicy(max_queue_size_),
                                                                           sub_input_filter_, sub_pcl_indices_filter_);
        sync_input_indices_a_->registerCallback(boost::bind(&ExpandSurfaces::input_callback, this, Surfaces::ConstPtr(), _1, _2));
    } else {
        sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer>(ExactPolicy(max_queue_size_),
                                                                          sub_input_filter_, sub_pcl_indices_filter_);
        sync_input_indices_e_->registerCallback(boost::bind(&ExpandSurfaces::input_callback, this, Surfaces::ConstPtr(), _1, _2));
    }

    sub_surfaces_.subscribe(*pnh_, "surfaces", max_queue_size_);
    surfaces_cache_ = boost::make_shared<message_filters::Cache<Surfaces>>(sub_surfaces_, 1);
    surfaces_cache_->registerCallback(boost::bind(&ExpandSurfaces::input_callback, this, _1, PointCloudIn::ConstPtr(), PointIndices::ConstPtr()));

    crophull_.setDim(2);
    crophull_.setCropOutside(true); // True returns only points inside the hull

    filter_inliers_.setDistance(perpendicular_dist_threshold_);

    latest_update_ = 0;

    NODELET_DEBUG("[%s::onInit] ExpandSurfaces Nodelet successfully created with connections:\n"
                  " - [subscriber] input             : %s\n"
                  " - [subscriber] indices           : %s\n"
                  " - [subscriber] surfaces          : %s\n"
                  " - [publisher]  expanded_segments : %s\n"
                  " - [publisher]  remaining_indices : %s\n"
                  " - [publisher]  filtered_indices  : %s\n"
                  " - [publisher]  removed_indices   : %s\n"
                  " - [publisher]  existing_inliers  : %s\n",
                  getName().c_str(), getMTPrivateNodeHandle().resolveName("input").c_str(),
                  getMTPrivateNodeHandle().resolveName("indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("surfaces").c_str(),
                  getMTPrivateNodeHandle().resolveName("expanded_segments").c_str(),
                  getMTPrivateNodeHandle().resolveName("remaining_indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("filtered_indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("removed_indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("existing_inliers").c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ExpandSurfaces::input_callback(const Surfaces::ConstPtr &surfaces_in,
                                                     const PointCloudIn::ConstPtr &cloud_in,
                                                     const PointIndices::ConstPtr &indices_in) {
    // pcl::ScopeTime scopetime("Expand Surfaces");

    if ((cloud_in && cloud_in->size() == 0) || (indices_in && indices_in->indices.size() == 0)) {
        return;
    }

    auto &surfaces = surfaces_in ? surfaces_in : surfaces_cache_->getElemBeforeTime(ros::Time::now());

    bool should_process = surfaces && (surfaces->latest_update >= latest_update_);
    if (indices_in && !should_process) {
        grouper_.add(cloud_in, indices_in);
    } else if (indices_in && surfaces) { // Note to future me: CLion says this is always true but it is WRONG
        auto point_group = grouper_.finish_group_with(cloud_in, indices_in);
        process(surfaces, point_group.first, point_group.second);
    } else if (should_process && surfaces) {
        process(surfaces, grouper_.finish_group(), nullptr);
    }
}

void surface_filters::ExpandSurfaces::process(const Surfaces::ConstPtr &surfaces, const PointCloudIn::ConstPtr &cloud,
                                              const PointIndices::ConstPtr &indices) {
    if (!surfaces || !cloud) return;

//    NODELET_INFO_STREAM_THROTTLE(5, "Current lag after change detection is "
//                                        << (ros::Time::now() - pcl_conversions::fromPCL(cloud->header.stamp))
//                                        << " seconds");

    if (cloud->size() == 0 || (indices && indices->indices.size() == 0)) {
        return;
    }

    HullCloudsMap hull_clouds = this->getHullCloudsMap(surfaces);

    //! TEMP: Extra radius outlier filter
    pcl::RadiusOutlierRemoval<PointIn> radius_filter;
    radius_filter.setInputCloud(cloud);
    if (indices) radius_filter.setIndices(indices);
    radius_filter.setRadiusSearch(parallel_dist_threshold_);
    radius_filter.setMinNeighborsInRadius(10);

    auto radius_indices = boost::make_shared<PointIndices>();
    radius_filter.filter(radius_indices->indices);

    pcl::IndicesPtr filtered_indices = filter_inliers_.getFilteredIndices(surfaces, cloud, radius_indices);
    pcl::IndicesPtr removed_indices = boost::make_shared<std::vector<int>>();

    {
        pcl::PointIndicesPtr filtered_indices_msg = boost::make_shared<pcl::PointIndices>();
        filtered_indices_msg->header = cloud->header;
        filtered_indices_msg->indices = *filtered_indices;
        pub_filtered_indices_.publish(filtered_indices_msg);
    }

    { // Publish the points that were removed because they were already inside a plane
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        PointCloudIn::Ptr cld = boost::make_shared<PointCloudIn>();
        extract.setInputCloud(cloud);
        extract.setIndices(filtered_indices);
        extract.setNegative(true);
        extract.filter(*cld);
        pub_existing_inliers_.publish(cld);
    }

    if (filtered_indices->size() == 0) {
        return;
    }

    pcl::search::KdTree<PointIn> search(false);
    search.setInputCloud(cloud, filtered_indices);

    for (const Surface &old_surface : surfaces->surfaces) {
        // pcl::ScopeTime for_scopetime(("Expand surface " + std::to_string(old_surface.id)).c_str());

        //
        // FILTER TO POINTS NEAR SURFACE BOUNDARY
        //
        pcl::IndicesPtr radius_filtered =
            filterWithinRadiusConnected(search, hull_clouds[old_surface.id], removed_indices);

        //        NODELET_DEBUG_STREAM("Found " << radius_filtered->size() << " indices near the edges of surface " <<
        //        old_surface.id);
        if (radius_filtered->size() == 0) continue;

        //
        // FILTER TO POINTS ON THE SURFACE
        //
        pcl::IndicesPtr distance_filtered =
            filter_inliers_.filterWithinModelDistance(cloud, radius_filtered, old_surface.model);
        //        NODELET_DEBUG_STREAM("Found " << distance_filtered->size() << " indices within the plane of surface "
        //        << old_surface.id);
        if (distance_filtered->size() == 0) continue;

        //
        // ADD NEW POINTS TO SURFACE
        //
        auto new_points_cloud = PointCloudIn(*cloud, *distance_filtered);
        new_points_cloud += old_surface.inliers;
        const Segment::Ptr new_segment =
            boost::make_shared<Segment>(cloud->header, old_surface.id, old_surface.model, new_points_cloud);
        this->latest_update_ = new_segment->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        this->pub_replace_surface_.publish(new_segment);

        //        NODELET_DEBUG_STREAM("[" << getName().c_str() << "::input_callback] ExpandSurface expanded surface "
        //                             << new_segment->surface_id << " from " << old_surface.inliers.size() << " to "
        //                             << new_segment->inliers.size() << " points ("
        //                             << (new_segment->inliers.size() - old_surface.inliers.size()) << " new points)");

        //
        // REMOVE THOSE POINTS FROM THE INPUT
        //
        removed_indices->reserve(removed_indices->size() + distance_filtered->size());
        std::copy(distance_filtered->begin(), distance_filtered->end(), std::back_inserter(*removed_indices));
        // Removed_indices' sortedness is a loop invariant
        // This could be replaced with an inplace_merge if distance_filtered is sorted
        std::sort(removed_indices->begin(), removed_indices->end());
    }

    {
        pcl::PointIndicesPtr removed_indices_msg = boost::make_shared<pcl::PointIndices>();
        removed_indices_msg->header = cloud->header;
        removed_indices_msg->indices = *removed_indices;
        pub_removed_indices_.publish(removed_indices_msg);
    }

    // Always publish remaining indices
    PointIndices::Ptr remaining_indices = boost::make_shared<PointIndices>();
    remaining_indices->header = cloud->header;
    remaining_indices->indices.reserve(filtered_indices->size() - removed_indices->size());
    std::sort(filtered_indices->begin(), filtered_indices->end());
    std::set_difference(filtered_indices->begin(), filtered_indices->end(), removed_indices->begin(),
                        removed_indices->end(), std::back_inserter(remaining_indices->indices));
    this->pub_remaining_indices_.publish(remaining_indices);
}
pcl::IndicesPtr
surface_filters::ExpandSurfaces::filterWithinRadiusConnected(const pcl::search::Search<PointIn> &search,
                                                             const PointCloudIn::Ptr &edge_points,
                                                             const pcl::IndicesPtr &removed_indices) const {
    // pcl::ScopeTime("Filter within radius !");
    std::set<int> within_radius_indices;

    std::queue<PointIn> to_search(std::deque<PointIn>(edge_points->begin(), edge_points->end()));

    while (!to_search.empty()) {
        const auto &point = to_search.front();

        std::vector<int> tmp_indices;
        std::vector<float> tmp_sqrdistances; // Only needed to fill a parameter

        search.radiusSearch(point, this->parallel_dist_threshold_, tmp_indices, tmp_sqrdistances);

        // TODO: Unify this with filterWithinHull by filtering tmp_indices here instead of later
        // Should speed this up because it won't explore too far from the surface in the perpendicular direction

        for (const auto &nearby_index : tmp_indices) {
            const auto insert_result = within_radius_indices.insert(nearby_index);

            if (insert_result.second) { // If the index didn't already exist in the list
                to_search.push(search.getInputCloud()->at(nearby_index));
            }
        }

        to_search.pop();
    }

    pcl::IndicesPtr within_radius_nodupes = boost::make_shared<std::vector<int>>();
    // This may reserve more space than necessary, but it shouldn't be much more.
    within_radius_nodupes->reserve(within_radius_indices.size());

    // Sortedness Invariants:
    // within_radius_indices is sorted because it's a set
    // removed_indices' sortedness is a loop invariant
    std::set_difference(within_radius_indices.begin(), within_radius_indices.end(), removed_indices->begin(),
                        removed_indices->end(), std::back_inserter(*within_radius_nodupes));

    return within_radius_nodupes;
}

auto surface_filters::ExpandSurfaces::getHullCloudsMap(const Surfaces::ConstPtr &surfaces) const -> HullCloudsMap {
    // pcl::ScopeTime("Get hull clouds");

    std::map<unsigned int, PointCloudIn::Ptr> hull_clouds;

    for (const Surface &surface : surfaces->surfaces) {
        hull_clouds[surface.id] = boost::make_shared<PointCloudIn>();
        fromPCLPointCloud2(surface.concave_hull.cloud, *hull_clouds[surface.id]);
    }

    return hull_clouds;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// void surface_filters::ExpandSurfaces::config_callback(ChangeDetectionConfig &config, uint32_t level) {
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

PLUGINLIB_EXPORT_CLASS(surface_filters::ExpandSurfaces, nodelet::Nodelet)
