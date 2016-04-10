/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/RegionGrowingSegmentation.h>

void surface_filters::RegionGrowingSegmentation::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Set up output publishers
    pub_output_ = pnh_->advertise<NormalCloudIn>("output", max_queue_size_);
    pub_clusters_ = pnh_->advertise<PointClusters>("clusters", max_queue_size_);

    // Check required parameters
    if (!pnh_->getParam("spatial_locator", spatial_locator_type_)) {
        NODELET_ERROR("[%s::onInit] Need a 'spatial_locator' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    // Enable the dynamic reconfigure service
    srv_ = boost::shared_ptr<dynamic_reconfigure::Server<RGSConfig>>(new dynamic_reconfigure::Server<RGSConfig>(*pnh_));
    srv_->setCallback(boost::bind(&RegionGrowingSegmentation::config_callback, this, _1, _2));

    // Optional parameters
    pnh_->getParam("use_indices", use_indices_);

    // Subscribe to the inputs using a filter
    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    sub_normals_filter_.subscribe(*pnh_, "normals", max_queue_size_);

    // TODO: Surely there's some way to do this without 4 copies of this stuff
    if (use_indices_) {
        // If indices are enabled, subscribe to the indices
        sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

        if (approximate_sync_) {
            sync_input_normals_indices_a_ =
                boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices>>(
                    max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_indices_a_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
            sync_input_normals_indices_a_->registerCallback(
                bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, _3));
        } else {
            sync_input_normals_indices_e_ =
                boost::make_shared<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn, PointIndices>>(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_indices_e_->connectInput(sub_input_filter_, sub_normals_filter_, sub_indices_filter_);
            sync_input_normals_indices_e_->registerCallback(
                bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, _3));
        }
    } else {
        if (approximate_sync_) {
            sync_input_normals_a_ =
                boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn>>(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_a_->connectInput(sub_input_filter_, sub_normals_filter_);
            sync_input_normals_a_->registerCallback(
                bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, PointIndicesConstPtr()));
        } else {
            sync_input_normals_e_ =
                boost::make_shared<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn>>(max_queue_size_);
            // surface not enabled, connect the input-indices duo and register
            sync_input_normals_e_->connectInput(sub_input_filter_, sub_normals_filter_);
            sync_input_normals_e_->registerCallback(
                bind(&RegionGrowingSegmentation::synchronized_input_callback, this, _1, _2, PointIndicesConstPtr()));
        }
    }

    NODELET_DEBUG("[%s::onInit] RegionGrowingSegmentation Nodelet successfully created with connections:\n"
                  " - [subscriber] input   : %s\n"
                  " - [subscriber] normals : %s\n"
                  " - [subscriber] indices : %s\n"
                  " - [publisher]  output  : %s\n"
                  " - [publisher]  clusters : %s\n",
                  getName().c_str(), getMTPrivateNodeHandle().resolveName("input").c_str(),
                  getMTPrivateNodeHandle().resolveName("normals").c_str(),
                  getMTPrivateNodeHandle().resolveName("indices").c_str(),
                  getMTPrivateNodeHandle().resolveName("output").c_str(),
                  getMTPrivateNodeHandle().resolveName("clusters").c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::RegionGrowingSegmentation::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
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

    /// DEBUG
    if (indices) {
        NODELET_DEBUG("[%s::input_indices_model_callback]\n"
                      "                                 - PointCloud with %d data points(%s), stamp %f, and frame %s "
                      "on topic %s received.\n"
                      "                                 - Normals PointCloud with %d data points(%s), stamp %f, and "
                      "frame %s on topic %s received.\n"
                      "                                 - PointIndices with %zu values, stamp %f, and frame %s on "
                      "topic %s received.",
                      getName().c_str(),

                      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
                      fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("input").c_str(),

                      normals->width * normals->height, pcl::getFieldsList(*normals).c_str(),
                      fromPCL(normals->header).stamp.toSec(), normals->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("normals").c_str(),

                      indices->indices.size(), pcl_conversions::fromPCL(indices->header.stamp).toSec(),
                      indices->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("indices").c_str());
    } else {
        NODELET_DEBUG(
            "[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
            getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
            cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());
    }

    ros::WallTime start = ros::WallTime::now();
    auto rgs = getLocalRGSObject();
    assert(&rgs != &impl_);

    rgs.setInputCloud(cloud);
    // Get a mutable version of Normals as demanded by RegionGrowingSegmentation
    rgs.setInputNormals(boost::make_shared<NormalCloudIn>(*normals));
    if (indices) rgs.setIndices(indices);

    NODELET_DEBUG("[%s::synchronized_input_callback] Running Region Growing Segmentation", getName().c_str());

    // Do the reconstruction
    rgs.extract(clusters->clusters);

    if (clusters->clusters.size() > 0) {
        if (should_populate_output) {
            output = rgs.getColoredCloud();
        }

        NODELET_INFO_STREAM(std::setprecision(3) << "(" << (ros::WallTime::now() - start) << " sec, " << output->size()
                                                 << " points, " << clusters->clusters.size() << " clusters) "
                                                 << "Region growing segmentation finished");
    } else {
        NODELET_INFO_STREAM(std::setprecision(3) << "(" << (ros::WallTime::now() - start) << " sec, " << cloud->size()
                                                 << " points) "
                                                 << "Region growing segmentation finished, no segments found");
    }

    // Enforce that the TF frame and the timestamp are copied
    output->header = cloud->header;
    pub_output_.publish(output);

    clusters->header = cloud->header;
    pub_clusters_.publish(clusters);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::RegionGrowingSegmentation::config_callback(RGSConfig &config,
                                                                 uint32_t level __attribute((unused))) {
    if (spatial_locator_type_ != config.spatial_locator) {
        spatial_locator_type_ = config.spatial_locator;
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
        impl_.setSmoothnessThreshold((float)config.smoothness_threshold);
        smoothness_threshold_ = config.smoothness_threshold;
        NODELET_DEBUG("[config_callback] Setting the smoothness threshold to: %f.", smoothness_threshold_);
    }

    if (residual_threshold_ != config.residual_threshold) {
        impl_.setResidualThreshold((float)config.residual_threshold);
        residual_threshold_ = config.residual_threshold;
        NODELET_DEBUG("[config_callback] Setting the residual threshold to: %f.", residual_threshold_);
    }

    if (curvature_threshold_ != config.curvature_threshold) {
        impl_.setCurvatureThreshold((float)config.curvature_threshold);
        curvature_threshold_ = config.curvature_threshold;
        NODELET_DEBUG("[config_callback] Setting the curvature threshold to: %f.", curvature_threshold_);
    }

    if (num_neighbors_ != config.number_of_neighbors) {
        impl_.setNumberOfNeighbours((unsigned int)config.number_of_neighbors);
        num_neighbors_ = config.number_of_neighbors;
        NODELET_DEBUG("[config_callback] Setting the number of neighbors to: %d.", num_neighbors_);
    }
}

auto surface_filters::RegionGrowingSegmentation::getLocalRGSObject() -> pcl::RegionGrowing<PointIn, NormalIn> {
    std::lock_guard<std::mutex> lock(setup_mutex_);

    // Make a copy of the data member
    auto rgs = impl_;

    // C++ will happily copy the kdtree's shared_ptr and leave it pointing to the same tree, and then two concurrent
    // copies of the class try to use it on different clouds at the same time, which causes :(.
    rgs.setSearchMethod(nullptr);

    return rgs;
}

PLUGINLIB_EXPORT_CLASS(surface_filters::RegionGrowingSegmentation, nodelet::Nodelet)
