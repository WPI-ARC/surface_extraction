/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/MovingLeastSquares.h>

void surface_filters::MovingLeastSquaresNodelet::setup_spatial_locator(int type) {
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

void surface_filters::MovingLeastSquaresNodelet::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Advertise output nodes
    pub_output_ = pnh_->advertise<PointCloudIn>("output", max_queue_size_);
    pub_normals_ = pnh_->advertise<NormalCloudOut>("normals", max_queue_size_);

    // Check for required parameters
    if (!pnh_->getParam("search_radius", search_radius_)) {
        NODELET_ERROR("[%s::onInit] Need a 'search_radius' parameter to be set before continuing!", getName().c_str());
        return;
    }
    if (!pnh_->getParam("spatial_locator", spatial_locator_type_)) {
        NODELET_ERROR("[%s::onInit] Need a 'spatial_locator' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    // Set required parameters in implementation
    impl_.setSearchRadius(search_radius_);
    setup_spatial_locator(spatial_locator_type_);

    // Enable the dynamic reconfigure service
//    srv_ = boost::shared_ptr<dynamic_reconfigure::Server<MLSConfig> >(new dynamic_reconfigure::Server<MLSConfig>(mutex_, *pnh_));
    srv_ = boost::make_shared<dynamic_reconfigure::Server<MLSConfig> >(*pnh_);
    srv_->setCallback(bind(&MovingLeastSquaresNodelet::config_callback, this, _1, _2));

    // Optional parameters
    pnh_->getParam("use_indices", use_indices_);

    if (use_indices_) {
        // If using indices, subscribe to the input and indices using a filter
        sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
        sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

        if (approximate_sync_) {
            sync_input_indices_a_ = boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, PointIndices> >(max_queue_size_);
            sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
            sync_input_indices_a_->registerCallback(
                    bind(&MovingLeastSquaresNodelet::synchronized_input_callback, this, _1, _2));
        } else {
            sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer<PointCloudIn, PointIndices> >(max_queue_size_);
            sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
            sync_input_indices_e_->registerCallback(
                    bind(&MovingLeastSquaresNodelet::synchronized_input_callback, this, _1, _2));
        }
    } else {
        // If not using indices, subscribe to the cloud directly
        sub_input_ = pnh_->subscribe<PointCloudIn>("input", max_queue_size_,
                                                   bind(&MovingLeastSquaresNodelet::synchronized_input_callback, this,
                                                        _1,
                                                        PointIndicesConstPtr()));
    }


    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                           " - use_indices    : %s\n"
                           " - spatial_locator: %d",
                   getName().c_str(),
                   (use_indices_) ? "true" : "false",
                   spatial_locator_type_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::MovingLeastSquaresNodelet::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                                             const PointIndices::ConstPtr &indices) {
    // No subscribers, no work
    if (pub_output_.getNumSubscribers() <= 0 && pub_normals_.getNumSubscribers() <= 0) {
        NODELET_DEBUG ("[%s::synchronized_input_callback] Input received but there are no subscribers; returning.",
                       getName().c_str());
        return;
    }

    // Create output objects
    PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();
    NormalCloudOut::Ptr normals = boost::make_shared<NormalCloudOut>();

    // TODO: Check that inputs are valid

    /// DEBUG
    if (indices) {
        NODELET_DEBUG("[%s::synchronized_input_callback]\n"
                              "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                      getName().c_str(),
                      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
                      fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("input").c_str(),
                      indices->indices.size(), indices->header.stamp.toSec(), indices->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("indices").c_str());
    } else {
        NODELET_DEBUG(
                "[%s::synchronized_input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
                getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
                cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());
    }

    // Ensure a spatial locator object exists
    if (!tree_) {
        NODELET_ERROR("[%s::synchronized_input_callback] MovingLeastSquares called without a valid spatial locator", getName().c_str());
        output->header = cloud->header;
        pub_output_.publish(output);
        return;
    }

    ros::WallTime start = ros::WallTime::now();

    // Create a non-const pointer to the indices and copy indices if used
    IndicesPtr indices_ptr;
    if (indices) indices_ptr.reset(new std::vector<int>(indices->indices));

    // Pass inputs to PCL
    impl_.setInputCloud(cloud);
    impl_.setIndices(indices_ptr);
    tree_->setInputCloud(cloud);
    impl_.setSearchMethod(tree_);

    NODELET_DEBUG(
            "[%s::synchronized_input_callback] Running MLS with search radius %f, polynomial fit %d, polynomial order %d, sqr gauss param %f, compute normals %d",
            getName().c_str(), impl_.getSearchRadius(), impl_.getPolynomialFit(), impl_.getPolynomialOrder(),
            impl_.getSqrGaussParam(), compute_normals_);

    // Do the reconstruction
    impl_.process(*normals);

    // Copy the points without normals into output to conform with nodelet contracts
    pcl::copyPointCloud(*normals, *output);

    NODELET_INFO_STREAM(std::setprecision(3) <<
                        "(" << (ros::WallTime::now() - start) << " sec, " << normals->size() << " points) "
                        << "MovingLeastSquares Normals Estimation Finished");

    // Publish outputs with the same header
    // NOTE: In order to benefit from zero-copy transport, output must be published as a boost::shared_ptr
    output->header = cloud->header;
    pub_output_.publish(output);

    if (compute_normals_) {
        normals->header = cloud->header;
        pub_normals_.publish(normals);
    }
}

void surface_filters::MovingLeastSquaresNodelet::config_callback(MLSConfig &config, uint32_t level __attribute__((unused))) {
    if (spatial_locator_type_ != config.spatial_locator) {
        spatial_locator_type_ = config.spatial_locator;
        // Initialize the spatial locator
        setup_spatial_locator(spatial_locator_type_);
        NODELET_DEBUG ("[config_callback] Setting the spatial locator to type: %d.", spatial_locator_type_);
    }
    if (search_radius_ != config.search_radius) {
        search_radius_ = config.search_radius;
        NODELET_DEBUG ("[config_callback] Setting the search radius: %f.", search_radius_);
        impl_.setSearchRadius(search_radius_);
        // This resets the sqr gauss parameter, so update the dynamic reconfigure service appropriately
        // TODO: Reenable if it didn't cause the crash
//        boost::recursive_mutex::scoped_lock __attribute__((unused)) scoped_lock(mutex_) ;
//        config.gaussian_parameter = sqrt(impl_.getSqrGaussParam());
//        srv_->updateConfig(config);
    }
    if (use_polynomial_fit_ != config.use_polynomial_fit) {
        use_polynomial_fit_ = config.use_polynomial_fit;
        NODELET_DEBUG ("[config_callback] Setting the use_polynomial_fit flag to: %d.", use_polynomial_fit_);
        impl_.setPolynomialFit(use_polynomial_fit_);
    }
    if (polynomial_order_ != config.polynomial_order) {
        polynomial_order_ = config.polynomial_order;
        NODELET_DEBUG ("[config_callback] Setting the polynomial order to: %d.", polynomial_order_);
        impl_.setPolynomialOrder(polynomial_order_);
    }
    if (gaussian_parameter_ != config.gaussian_parameter) {
        gaussian_parameter_ = config.gaussian_parameter;
        NODELET_DEBUG ("[config_callback] Setting the gaussian parameter to: %f.", gaussian_parameter_);
        impl_.setSqrGaussParam(gaussian_parameter_ * gaussian_parameter_);
    }
    if (compute_normals_ != config.compute_normals) {
        compute_normals_ = config.compute_normals;
        NODELET_DEBUG ("[config_callback] Setting the compute normals flag to: %d.", compute_normals_);
        impl_.setComputeNormals(compute_normals_);
    }
}

PLUGINLIB_EXPORT_CLASS(surface_filters::MovingLeastSquaresNodelet, nodelet::Nodelet)
