/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ConcaveHull.h>

void surface_filters::ConcaveHull::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Advertise output nodes
    pub_output_ = pnh_->advertise<pcl_msgs::PolygonMesh>("output", max_queue_size_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<ConcaveHullConfig> >(*pnh_);
    srv_->setCallback(bind(&ConcaveHull::config_callback, this, _1, _2));

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
                    bind(&ConcaveHull::synchronized_input_callback, this, _1, _2));
        } else {
            sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer<PointCloudIn, PointIndices> >(max_queue_size_);
            sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
            sync_input_indices_e_->registerCallback(
                    bind(&ConcaveHull::synchronized_input_callback, this, _1, _2));
        }
    } else {
        // If not using indices, subscribe to the cloud directly
        sub_input_ = pnh_->subscribe<PointCloudIn>("input", max_queue_size_,
                                                   bind(&ConcaveHull::synchronized_input_callback, this,
                                                        _1,
                                                        PointIndicesConstPtr()));
    }


    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                           " - use_indices    : %s\n",
                   getName().c_str(),
                   (use_indices_) ? "true" : "false");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ConcaveHull::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                               const PointIndices::ConstPtr &indices) {
    // No subscribers, no work
    if (pub_output_.getNumSubscribers() <= 0) {
        NODELET_DEBUG ("[%s::synchronized_input_callback] Input received but there are no subscribers; returning.",
                       getName().c_str());
        return;
    }

    // Create output objects
    PolygonMesh::Ptr concave_hull = boost::make_shared<PolygonMesh>();

    // TODO: Check that inputs are valid

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

    std::unique_lock<std::mutex> hull_lock(hull_mutex_);

    ros::WallTime start = ros::WallTime::now();

    // Pass inputs to PCL
    impl_.setInputCloud(cloud);
    if (use_indices_ && indices != NULL) {
        impl_.setIndices(boost::make_shared<std::vector<int> >(indices->indices));
    }

    // Do the reconstruction
    impl_.reconstruct(*concave_hull);

    hull_lock.unlock();

    auto num_pts = concave_hull->cloud.height * concave_hull->cloud.width;
    NODELET_INFO_STREAM("[" << getName().c_str() << "::synchronized_input_callback] " << std::setprecision(3) <<
                        "(" << (ros::WallTime::now() - start) << " sec, " << num_pts << " points) "
                        << "Concave Hull Finished");

    // Publish outputs with the same header
    // NOTE: In order to benefit from zero-copy transport, output must be published as a boost::shared_ptr
    concave_hull->header = cloud->header;
    pcl_msgs::PolygonMesh::Ptr concave_hull_ros = boost::make_shared<pcl_msgs::PolygonMesh>();
    pcl_conversions::fromPCL(*concave_hull, *concave_hull_ros);
    pub_output_.publish(concave_hull_ros);
}

void surface_filters::ConcaveHull::config_callback(ConcaveHullConfig &config, uint32_t level __attribute__((unused))) {
    if (dimension_ != config.dimension) {
        dimension_ = config.dimension;
        NODELET_DEBUG ("[config_callback] Setting the dimension to: %d.", dimension_);
        impl_.setDimension(dimension_);
    }
    if (alpha_ != config.alpha) {
        alpha_ = config.alpha;
        NODELET_DEBUG ("[config_callback] Setting the alpha parameter to: %f.", alpha_);
        impl_.setAlpha(alpha_);
    }
}

PLUGINLIB_EXPORT_CLASS(surface_filters::ConcaveHull, nodelet::Nodelet)
