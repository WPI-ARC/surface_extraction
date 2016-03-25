//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ExpandSurfaces.h>
#include <pcl/common/time.h>
//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::ExpandSurfaces::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_replace_surface_ = pnh_->advertise<Segment>("expanded_segments", max_queue_size_);
    pub_remaining_indices_ = pnh_->advertise<PointIndices>("removed_indices", max_queue_size_);
    pub_filtered_indices_ = pnh_->advertise<PointIndices>("filtered_indices", max_queue_size_);
    pub_removed_indices_ = pnh_->advertise<PointIndices>("removed_indices", max_queue_size_);

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

    if (approximate_sync_) {
        sync_input_indices_a_ = boost::make_shared<ApproxTimeSynchronizer>(ApproxPolicy(max_queue_size_),
                                                                           sub_input_filter_, sub_pcl_indices_filter_);
        sync_input_indices_a_->registerCallback(bind(&ExpandSurfaces::synchronized_input_callback, this, _1, _2));
    } else {
        sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer>(ExactPolicy(max_queue_size_),
                                                                          sub_input_filter_, sub_pcl_indices_filter_);
        sync_input_indices_e_->registerCallback(bind(&ExpandSurfaces::synchronized_input_callback, this, _1, _2));
    }

    sub_surfaces_.subscribe(*pnh_, "surfaces", max_queue_size_);
    surfaces_cache_ = boost::make_shared<message_filters::Cache<Surfaces> >(sub_surfaces_, 1);
    
//    sub_surface_clouds_.subscribe(*pnh_, "surface_clouds", max_queue_size_);
//    surface_clouds_cache_ = boost::make_shared<message_filters::Cache<SurfaceClouds> >(sub_surface_clouds_, 1);

    crophull_.setDim(2);
    crophull_.setCropOutside(true); // True returns only points inside the hull

    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following subscriptions:\n"
                           " - input    : %s\n"
                           " - indices    : %s\n"
                           " - surfaces    : %s\n",
                   getName().c_str(),
                   getMTPrivateNodeHandle().resolveName("input").c_str(),
                   getMTPrivateNodeHandle().resolveName("indices").c_str(),
                   getMTPrivateNodeHandle().resolveName("surfaces").c_str());
}


//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ExpandSurfaces::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud_in,
                                                                  const PointIndices::ConstPtr &indices_in) {
    //pcl::ScopeTime scopetime("Expand Surfaces");

    if (cloud_in->width * cloud_in->height == 0 || indices_in->indices.size() == 0) {
        return;
    }

    /// DEBUG
//    NODELET_DEBUG("[%s::synchronized_input_callback]\n"
//                          "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
//                          "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
//                  getName().c_str(),
//                  cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
//                  fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
//                  getMTPrivateNodeHandle().resolveName("input").c_str(),
//                  indices->indices.size(), indices->header.stamp.toSec(), indices->header.frame_id.c_str(),
//                  getMTPrivateNodeHandle().resolveName("indices").c_str());

    Surfaces::ConstPtr surfaces = surfaces_cache_->getElemBeforeTime(ros::Time::now());

    PointCloudIn::Ptr cloud;

    pcl::StopWatch lock_timer;
    std::unique_lock<std::mutex> pending_points_lock(pending_points_mutex_);
    auto lock_time = lock_timer.getTime();

    if (!surfaces) { // If no surfaces yet, add this to the set of pending points and return
        std::size_t prev_num_pending_points = pending_points_ ? pending_points_->size() : 0;
        if (!pending_points_) { // If no pending points yet, make one
            pending_points_ = indices_in.get() != nullptr ? boost::make_shared<PointCloudIn>(*cloud_in, indices_in->indices) :
                                           boost::make_shared<PointCloudIn>(*cloud_in);
        } else { // If there is an existing pending points cloud,
            *pending_points_ += indices_in.get() != nullptr ? PointCloudIn(*cloud_in, indices_in->indices) : PointCloudIn(*cloud_in);
        }
        NODELET_DEBUG_STREAM("ExpandSurfaces waiting for surfaces list (" << pending_points_->size() <<
                             " pending points, " << (pending_points_->size() - prev_num_pending_points) <<
                             " new) (locked for " << lock_time << "ms)");
        return;
    } else if (pending_points_) { // If there are pending points to process, add the new points to them
        *pending_points_ += indices_in.get() != nullptr ? PointCloudIn(*cloud_in, indices_in->indices) : PointCloudIn(*cloud_in);
    }

    if (pending_points_) {
        // If pending points exists, use std::swap to put the pending points into cloud and a null sharedptr into
        // pending points so that as soon as pending_points_lock is released, another thread can use pending_points.
        std::swap(pending_points_, cloud);
    }

    pending_points_lock.unlock();

    if (cloud) {
        this->process(surfaces, cloud, PointIndices::Ptr());
    } else {
        this->process(surfaces, cloud_in, indices_in);
    }
}

void surface_filters::ExpandSurfaces::process(const Surfaces::ConstPtr &surfaces,
                                              const PointCloudIn::ConstPtr &cloud,
                                              const PointIndices::ConstPtr &indices) {
    //pcl::ScopeTime scopetime("Expand surfaces processing");
    NODELET_INFO_STREAM_THROTTLE(5 /*seconds*/, "Current lag after change detection is "
                                                << (ros::Time::now() - pcl_conversions::fromPCL(cloud->header.stamp))
                                                << " seconds");


    if (cloud->size() == 0 || (indices && indices->indices.size() == 0)) {
        return;
    }

    HullCloudsMap hull_clouds = this->getHullCloudsMap(surfaces);

    pcl::IndicesPtr filtered_indices = this->getFilteredIndices(surfaces, cloud, indices, hull_clouds);
    pcl::IndicesPtr removed_indices = boost::make_shared<std::vector<int> >();

    {
        pcl::PointIndicesPtr filtered_indices_msg = boost::make_shared<pcl::PointIndices>();
        filtered_indices_msg->header = cloud->header;
        filtered_indices_msg->indices = *filtered_indices;
        pub_filtered_indices_.publish(filtered_indices_msg);
    }

    if (filtered_indices->size() == 0) {
        return;
    }

    pcl::search::KdTree<PointIn> search(false);
    search.setInputCloud(cloud, filtered_indices);

    for (const Surface &old_surface : surfaces->surfaces) {
        //pcl::ScopeTime for_scopetime(("Expand surface " + std::to_string(old_surface.id)).c_str());

        //
        // FILTER TO POINTS NEAR SURFACE BOUNDARY
        //
        pcl::IndicesPtr radius_filtered = filterWithinRadiusConnected(search, hull_clouds[old_surface.id],
                                                                      removed_indices);

        NODELET_DEBUG_STREAM("Found " << radius_filtered->size() << " indices near the edges of surface " << old_surface.id);
        if (radius_filtered->size() == 0) continue;

        //
        // FILTER TO POINTS ON THE SURFACE
        //
        pcl::IndicesPtr distance_filtered = filterWithinModelDistance(cloud, radius_filtered, old_surface.model);
        NODELET_DEBUG_STREAM("Found " << distance_filtered->size() << " indices within the plane of surface " << old_surface.id);
        if (distance_filtered->size() == 0) continue;

        //
        // ADD NEW POINTS TO SURFACE
        //
        auto new_points_cloud = PointCloudIn(*cloud, *distance_filtered);
        new_points_cloud += old_surface.inliers;
        const Segment::Ptr new_segment = boost::make_shared<Segment>(cloud->header, old_surface.id,
                                                                     old_surface.model, new_points_cloud);
        this->pub_replace_surface_.publish(new_segment);

        NODELET_DEBUG_STREAM("[" << getName().c_str() << "::input_callback] ExpandSurface expanded surface "
                             << new_segment->surface_id << " from " << old_surface.inliers.size() << " to "
                             << new_segment->inliers.size() << " points ("
                             << (new_segment->inliers.size() - old_surface.inliers.size()) << " new points)");

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
    std::set_difference(filtered_indices->begin(), filtered_indices->end(),
                        removed_indices->begin(), removed_indices->end(),
                        std::back_inserter(remaining_indices->indices));
    this->pub_remaining_indices_.publish(remaining_indices);
}

pcl::IndicesPtr surface_filters::ExpandSurfaces::filterWithinModelDistance(const PointCloudIn::ConstPtr &input,
                                                                           const pcl::IndicesConstPtr &indices,
                                                                           const pcl::ModelCoefficients &coeff) {
    //pcl::ScopeTime("Filter within model distance");

    auto model = pcl::SampleConsensusModelPlane<PointIn>(input, *indices);

    pcl::IndicesPtr output_indices = boost::make_shared<std::vector<int> >();
    model.selectWithinDistance(Eigen::Vector4f(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]),
                               perpendicular_dist_threshold_, *output_indices);
    return output_indices;
}

pcl::IndicesPtr surface_filters::ExpandSurfaces::filterWithinHull(const PointCloudIn::ConstPtr &input,
                                                                  const pcl::IndicesConstPtr &indices,
                                                                  const PointCloudIn::Ptr &hull_cloud, // Can't be Const
                                                                  const std::vector<pcl::Vertices> &hull_polygons) {
    //pcl::ScopeTime("Filter within hull");

    this->crophull_.setHullCloud(hull_cloud);
    this->crophull_.setHullIndices(hull_polygons);
    this->crophull_.setInputCloud(input);
    this->crophull_.setIndices(indices);

    pcl::IndicesPtr output_indices = boost::make_shared<std::vector<int> >();
    this->crophull_.filter(*output_indices);

    return output_indices;
}

pcl::IndicesPtr surface_filters::ExpandSurfaces::filterWithinRadiusConnected(const pcl::search::Search<PointIn> &search,
                                                                             const PointCloudIn::Ptr &edge_points,
                                                                             const pcl::IndicesPtr &removed_indices) const {
    //pcl::ScopeTime("Filter within radius !");
    std::set<int> within_radius_indices;

    std::queue<PointIn> to_search(std::deque<PointIn>(edge_points->begin(), edge_points->end()));

    while (!to_search.empty()) {
        const auto &point = to_search.front();

        std::vector<int> tmp_indices;
        std::vector<float> tmp_sqrdistances; // Only needed to fill a parameter

        search.radiusSearch(point, this->parallel_dist_threshold_, tmp_indices, tmp_sqrdistances);

        for (const auto &nearby_index : tmp_indices) {
            const auto insert_result = within_radius_indices.insert(nearby_index);

            if (insert_result.second) { // If the index didn't already exist in the list
                to_search.push(search.getInputCloud()->at(nearby_index));
            }
        }

        to_search.pop();
    }

    pcl::IndicesPtr within_radius_nodupes = boost::make_shared<std::vector<int> >();
    // This may reserve more space than necessary, but it shouldn't be much more.
    within_radius_nodupes->reserve(within_radius_indices.size());

    // Sortedness Invariants:
    // within_radius_indices is sorted because it's a set
    // removed_indices' sortedness is a loop invariant
    std::set_difference(within_radius_indices.begin(), within_radius_indices.end(),
                        removed_indices->begin(), removed_indices->end(),
                        std::back_inserter(*within_radius_nodupes));

    return within_radius_nodupes;
}

pcl::IndicesPtr surface_filters::ExpandSurfaces::getFilteredIndices(const Surfaces::ConstPtr &surfaces,
                                                                    const PointCloudIn::ConstPtr &cloud_in,
                                                                    const PointIndices::ConstPtr &indices,
                                                                    const HullCloudsMap &hull_clouds) {
    //pcl::ScopeTime("Filtering indices");

    pcl::IndicesPtr indices_remaining = indices ? boost::make_shared<std::vector<int> >(indices->indices)
                                                : boost::make_shared<std::vector<int> >();

    std::sort(indices_remaining->begin(), indices_remaining->end());
    for (const Surface &surface : surfaces->surfaces) {
        // Clip point cloud to plane
        pcl::IndicesPtr indices_inplane = this->filterWithinModelDistance(cloud_in, indices_remaining, surface.model);

//        NODELET_DEBUG_STREAM("Found " << indices_inplane->size() << " indices in the plane of surface " << surface.id);
        if (indices_inplane->size() == 0) continue;

        // Clip point cloud to be within hull
        pcl::IndicesPtr indices_inhull = this->filterWithinHull(cloud_in, indices_inplane,  hull_clouds.at(surface.id),
                                                                surface.concave_hull.polygons);

//        NODELET_DEBUG_STREAM("Found " << indices_inhull->size() << " indices in the hull of surface " << surface.id);
        if (indices_inplane->size() == 0) continue;

        std::sort(indices_inhull->begin(), indices_inhull->end());

        pcl::IndicesPtr indices_remaining_tmp = boost::make_shared<std::vector<int> >();
        std::set_difference(indices_remaining->begin(), indices_remaining->end(),
                            indices_inhull->begin(), indices_inhull->end(),
                            std::back_inserter(*indices_remaining_tmp));

        indices_remaining.swap(indices_remaining_tmp);

//        NODELET_DEBUG_STREAM("Now " << indices_remaining->size() << " indices remaining");
    }

//    NODELET_DEBUG_STREAM("Filtered indices from " << (indices ? indices->indices.size() : cloud_in->size())
//                         << " to " << indices_remaining->size());

    return indices_remaining;
}

auto surface_filters::ExpandSurfaces::getHullCloudsMap(const Surfaces::ConstPtr &surfaces) const -> HullCloudsMap {
    //pcl::ScopeTime("Get hull clouds");

    std::map<unsigned int, PointCloudIn::Ptr> hull_clouds;

    for (const Surface &surface : surfaces->surfaces) {
        hull_clouds[surface.id] = boost::make_shared<PointCloudIn>();
        fromPCLPointCloud2(surface.concave_hull.cloud, *hull_clouds[surface.id]);
    }

    return hull_clouds;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//void surface_filters::ExpandSurfaces::config_callback(ChangeDetectionConfig &config, uint32_t level) {
//    if (resolution_ != config.resolution) {
//        resolution_ = config.resolution;
//        hull_ = ChangeDetector(resolution_);
//        NODELET_DEBUG("[config_callback] Setting the resolution to: %f. Note that changing the resolution requires restarting change detection", resolution_);
//    }
//
//    if (min_points_in_leaf_ != config.min_points_per_leaf) {
//        min_points_in_leaf_ = config.min_points_per_leaf;
//        NODELET_DEBUG("[config_callback] Setting the max radius to: %d.", min_points_in_leaf_);
//    }
//}

PLUGINLIB_EXPORT_CLASS(surface_filters::ExpandSurfaces, nodelet::Nodelet)
