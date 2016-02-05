/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/RegionGrowingSegmentation.h>

void surface_filters::RegionGrowingSegmentation::setup_spatial_locator(int type) {
    switch (type) {
        // case 0: As far as I can tell, ANN is no longer available
        case 1:
            tree_ = SpatialSearch::Ptr(new pcl::search::KdTree<PointIn>());
            break;
        case 2:
            tree_ = SpatialSearch::Ptr(new pcl::search::OrganizedNeighbor<PointIn>());
            break;
        default:
            NODELET_ERROR("[%s::synchronized_input_callback] Invalid spatial locator type %d", getName().c_str(), type);
            tree_ = SpatialSearch::Ptr();
    }
}

void surface_filters::RegionGrowingSegmentation::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Set up output publishers
    pub_output_ = pnh_->advertise<NormalCloudIn>("output", (uint32_t) max_queue_size_);
    pub_clusters_ = pnh_->advertise<PointClusters>("clusters", (uint32_t) max_queue_size_);

    // Check required parameters
    if (!pnh_->getParam("spatial_locator", spatial_locator_type_)) {
        NODELET_ERROR("[%s::onInit] Need a 'spatial_locator' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    // Required setup
    setup_spatial_locator(spatial_locator_type_);

    // Enable the dynamic reconfigure service
    srv_ = boost::shared_ptr<dynamic_reconfigure::Server<RGSConfig> >(new dynamic_reconfigure::Server<RGSConfig>(*pnh_));
    auto f = boost::bind(&RegionGrowingSegmentation::config_callback, this, _1, _2);
    srv_->setCallback((const dynamic_reconfigure::Server<surface_filters::RGSConfig>::CallbackType &) f);

    // Optional parameters
    pnh_->getParam("use_indices", use_indices_);

    // Subscribe to the inputs using a filter
    sub_input_filter_.subscribe(*pnh_, "input", 1);
    sub_normals_filter_.subscribe(*pnh_, "normals", 1);

    // TODO: Surely there's some way to do this without 4 copies of this stuff
    if (use_indices_) {
        // If indices are enabled, subscribe to the indices
        sub_indices_filter_.subscribe(*pnh_, "indices", 1);

        if (approximate_sync_) {
            sync_input_normals_indices_a_ = boost::make_shared<ApproximateTimeSynchronizer<
                    PointCloudIn, NormalCloudIn, PointIndices> >(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_indices_a_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
            sync_input_normals_indices_a_->registerCallback(
                    bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, _3));
        }
        else {
            sync_input_normals_indices_e_ = boost::make_shared<ExactTimeSynchronizer<
                    PointCloudIn, NormalCloudIn, PointIndices> >(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_indices_e_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
            sync_input_normals_indices_e_->registerCallback(
                    bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, _3));
        }
    } else {
        if (approximate_sync_) {
            sync_input_normals_a_ = boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn> >
                    (max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_a_->connectInput(sub_input_filter_, sub_normals_filter_);
            sync_input_normals_a_->registerCallback(
                    bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2,
                         PointIndicesConstPtr()));
        }
        else {
            sync_input_normals_e_ = boost::make_shared<ExactTimeSynchronizer<
                    PointCloudIn, NormalCloudIn> >(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_e_->connectInput(sub_input_filter_, sub_normals_filter_);
            sync_input_normals_e_->registerCallback(
                    bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2,
                         PointIndicesConstPtr()));
        }
    }


    NODELET_DEBUG("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                   " - use_indices    : %s",
                   getName().c_str(),
                  (use_indices_) ? "true" : "false");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
surface_filters::RegionGrowingSegmentation::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                                        const NormalCloudIn::ConstPtr &normals,
                                                                        const PointIndices::ConstPtr &indices) {
    // No subscribers, no work
    if (pub_output_.getNumSubscribers() <= 0 && pub_clusters_.getNumSubscribers() <= 0) {
        NODELET_DEBUG("[%s::input_callback] Input relieved but there are no subscribers; returning.",
                      getName().c_str());
        return;
    }

    bool should_populate_output = pub_output_.getNumSubscribers() > 0;

    // Make output objects
    PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();
    surfaces::PointClusters::Ptr clusters = boost::make_shared<surfaces::PointClusters>();

    // If cloud is given, check if it's valid
    if (!isValid(cloud)) {
        NODELET_ERROR("[%s::synchronized_input_callback] Invalid input!", getName().c_str());
        output->header = cloud->header;
        pub_output_.publish(output);
        return;
    }
    // If indices are given, check if they are valid
    if (indices && !isValid(indices, "indices")) {
        NODELET_ERROR("[%s::synchronized_input_callback] Invalid indices!", getName().c_str());
        output->header = cloud->header;
        pub_output_.publish(output);
        return;
    }

    /// DEBUG
    if (indices) {
        NODELET_DEBUG("[%s::input_indices_model_callback]\n"
                              "                                 - PointCloud with %d data points(%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - Normals PointCloud with %d data points(%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                      getName().c_str(),

                      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
                      fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("input").c_str(),

                      normals->width * normals->height, pcl::getFieldsList(*normals).c_str(),
                      fromPCL(normals->header).stamp.toSec(), normals->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("normals").c_str(),

                      indices->indices.size(), indices->header.stamp.toSec(), indices->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("indices").c_str());
    } else {
        NODELET_DEBUG(
                "[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
                getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
                cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());
    }

    ros::WallTime start = ros::WallTime::now();

    // Ensure a spatial locator object exists
    if (!tree_) {
        NODELET_ERROR("[%s::synchronized_input_callback] MovingLeastSquares called without a valid spatial locator", getName().c_str());
        output->header = cloud->header;
        pub_output_.publish(output);
        return;
    }

    // Get a mutable version of Normals as demanded by RegionGrowingSegmentation
    NormalCloudIn::Ptr normals_mutable = boost::make_shared<NormalCloudIn>(*normals);
    impl_.setInputCloud(cloud);
    impl_.setInputNormals(normals_mutable);

    IndicesPtr indices_ptr;
    if (indices) indices_ptr.reset(new std::vector<int>(indices->indices));

    impl_.setIndices(indices_ptr);

    tree_->setInputCloud(cloud);
    impl_.setSearchMethod(tree_);

    NODELET_DEBUG("[%s::synchronized_input_callback] Running Region Growing Segmentation", getName().c_str());

    // Do the reconstruction
    impl_.extract(clusters->clusters);

    if (clusters->clusters.size() > 0) {
        if (should_populate_output) {
            output = impl_.getColoredCloud();

            // TODO: This shouldn't happen
            if (output->size() == 0) {
                output = boost::make_shared<PointCloudOut>();
            }
        }

        NODELET_INFO_STREAM(std::setprecision(3) <<
                            "(" << (ros::WallTime::now() - start) << " sec, " << output->size() << " points, " << clusters->clusters.size() << " clusters) "
                            << "Region growing segmentation finished");
    } else {
        NODELET_INFO_STREAM(std::setprecision(3) <<
                            "(" << (ros::WallTime::now() - start) << " sec, " << cloud->size() << " points) "
                            << "Region growing segmentation finished, no segments found");
    }

    // Publish a Boost shared ptr const data
    // Enforce that the TF frame and the timestamp are copied
    output->header = cloud->header;
    pub_output_.publish(output);

    clusters->header = cloud->header;
    pub_clusters_.publish(clusters);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::RegionGrowingSegmentation::config_callback(RGSConfig &config, uint32_t level __attribute((unused))) {
    if (spatial_locator_type_ != config.spatial_locator) {
        spatial_locator_type_ = config.spatial_locator;
        setup_spatial_locator(spatial_locator_type_);
        NODELET_DEBUG("[config_callback] Setting the spatial locator to type: %d.", spatial_locator_type_);
    }

    if (min_cluster_size_ != config.min_cluster_size) {
        impl_.setMinClusterSize(config.min_cluster_size);
        min_cluster_size_ = config.min_cluster_size;
        NODELET_DEBUG("[config_callback] Setting the minimum cluster size to: %d.", min_cluster_size_);
    }

    if (max_cluster_size_ != config.max_cluster_size) {
        impl_.setMaxClusterSize(config.max_cluster_size);
        max_cluster_size_ = config.max_cluster_size;
        NODELET_DEBUG("[config_callback] Setting the maximum cluster size to: %d.", max_cluster_size_);
    }

    if (smooth_mode_flag_ != config.smooth_mode) {
        impl_.setSmoothModeFlag(config.smooth_mode);
        smooth_mode_flag_ = config.smooth_mode;
        NODELET_DEBUG("[config_callback] Setting the smooth mode flag to: %d.", smooth_mode_flag_);
    }

    if (curvature_test_flag_ != config.curvature_test) {
        impl_.setCurvatureTestFlag(config.curvature_test);
        curvature_test_flag_ = config.curvature_test;
        NODELET_DEBUG("[config_callback] Setting curvature test flag to: %d.", curvature_test_flag_);
    }

    if (residual_test_flag_ != config.residual_test) {
        impl_.setResidualTestFlag(config.residual_test);
        residual_test_flag_ = config.residual_test;
        NODELET_DEBUG("[config_callback] Setting the residual test flag to: %d.", residual_test_flag_);
    }

    if (smoothness_threshold_ != config.smoothness_threshold) {
        impl_.setSmoothnessThreshold((float) config.smoothness_threshold);
        smoothness_threshold_ = config.smoothness_threshold;
        NODELET_DEBUG("[config_callback] Setting the smoothness threshold to: %f.", smoothness_threshold_);
    }

    if (residual_threshold_ != config.residual_threshold) {
        impl_.setResidualThreshold((float) config.residual_threshold);
        residual_threshold_ = config.residual_threshold;
        NODELET_DEBUG("[config_callback] Setting the residual threshold to: %f.", residual_threshold_);
    }

    if (curvature_threshold_ != config.curvature_threshold) {
        impl_.setCurvatureThreshold((float) config.curvature_threshold);
        curvature_threshold_ = config.curvature_threshold;
        NODELET_DEBUG("[config_callback] Setting the curvature threshold to: %f.", curvature_threshold_);
    }

    if (num_neighbors_ != config.number_of_neighbors) {
        impl_.setNumberOfNeighbours((unsigned int) config.number_of_neighbors);
        num_neighbors_ = config.number_of_neighbors;
        NODELET_DEBUG("[config_callback] Setting the number of neighbors to: %d.", num_neighbors_);
    }
}

PLUGINLIB_EXPORT_CLASS(surface_filters::RegionGrowingSegmentation, nodelet::Nodelet)
