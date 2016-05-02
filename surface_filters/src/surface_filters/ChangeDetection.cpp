/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ChangeDetection.h>
#include <pcl/io/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::ChangeDetection::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_output_ = pnh_->advertise<PointCloudOut>("output", max_queue_size_);
    pub_new_scene_ = pnh_->advertise<PointCloudOut>("new_scene", max_queue_size_);
    pub_indices_ = pnh_->advertise<PointIndices>("new_indices", max_queue_size_);

    if (!pnh_->getParam("resolution", resolution_)) {
        NODELET_ERROR("[%s::onInit] Need a 'resolution' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    impl_ = ChangeDetector(resolution_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<ChangeDetectionConfig> >(*pnh_);
    srv_->setCallback(boost::bind(&ChangeDetection::config_callback, this, _1, _2));

    // Subscribe to the input directly b/c there is nothing to synchronize
    sub_input_ = pnh_->subscribe<PointCloudIn>("input", max_queue_size_, bind(&ChangeDetection::synchronized_input_callback, this, _1));

    NODELET_DEBUG ("[%s::onInit] ChangeDetection Nodelet successfully created with connections:\n"
                           " - [subscriber] input       : %s\n"
                           " - [publisher]  output      : %s\n"
                           " - [publisher]  new_scene   : %s\n"
                           " - [publisher]  new_indices : %s\n",
                   getName().c_str(),
                   getMTPrivateNodeHandle().resolveName("input").c_str(),
                   getMTPrivateNodeHandle().resolveName("output").c_str(),
                   getMTPrivateNodeHandle().resolveName("new_scene").c_str(),
                   getMTPrivateNodeHandle().resolveName("new_indices").c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ChangeDetection::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud) {
    // No subscribers, no work
    if (pub_indices_.getNumSubscribers() <= 0 && pub_output_.getNumSubscribers() <= 0 && pub_new_scene_.getNumSubscribers() <= 0) {
        NODELET_DEBUG("[%s::input_callback] Input received but there are no subscribers; returning.",
                      getName().c_str());
        return;
    }
    NODELET_INFO_STREAM_ONCE("======>{\"event\": \"first_scan\", \"time\": " << ros::Time::now() << "}");


    NODELET_INFO_STREAM("======>{\"event\": \"scan_recieved\", \"value\": " << ros::Time::now() << ", \"cloud_stamp\": " << cloud->header.stamp << "}");

    // TODO: If cloud is given, check if it's valid

    /// DEBUG
//    NODELET_DEBUG("[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
//                  getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
//                  cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());

    ros::WallTime start = ros::WallTime::now();

    impl_.setInputCloud(cloud);
    impl_.addPointsFromInputCloud();

    pcl::PointIndicesPtr indices = boost::make_shared<pcl::PointIndices>();

    impl_.getPointIndicesFromNewVoxels(indices->indices, min_points_in_leaf_);

    // Swap in preparation for the next call
    impl_.switchBuffers();

    float new_percentage = indices->indices.size() / (float) cloud->size();

    if (new_percentage > 0) {
//        NODELET_DEBUG_STREAM(std::setprecision(3) <<
//                             "(" << (ros::WallTime::now() - start) << " sec, " << cloud->size() << " points) "
//                             << "Change Detection Finished with " << indices->indices.size()
//                             << " new points (" << new_percentage*100 <<"%)");
    }

    if (new_percentage >= new_scene_threshold_) {
        // Then it's a new scene -- republish the cloud
        pub_new_scene_.publish(cloud);
    } else {
        indices->header = cloud->header;
        pub_indices_.publish(indices);
    }

    NODELET_INFO_STREAM("======>{\"event\": \"scan_size\", \"value\": " << indices->indices.size() << ", \"cloud_stamp\": " << cloud->header.stamp << "}");

    // Always publish output
    pub_output_.publish(cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ChangeDetection::config_callback(ChangeDetectionConfig &config, uint32_t level) {
    if (resolution_ != config.resolution) {
        resolution_ = config.resolution;
        impl_ = ChangeDetector(resolution_);
        NODELET_DEBUG("[config_callback] Setting the resolution to: %f. Note that changing the resolution requires restarting change detection", resolution_);
    }

    if (min_points_in_leaf_ != config.min_points_per_leaf) {
        min_points_in_leaf_ = config.min_points_per_leaf;
        NODELET_DEBUG("[config_callback] Setting the max radius to: %d.", min_points_in_leaf_);
    }
}

PLUGINLIB_EXPORT_CLASS(surface_filters::ChangeDetection, nodelet::Nodelet)

