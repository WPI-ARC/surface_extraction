/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/SamplePatches.h>
#include <pcl/io/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::SamplePatches::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_output_ = pnh_->advertise<PointClusters>("output", max_queue_size_);

    if (!pnh_->getParam("spatial_locator", spatial_locator_type_)) {
        NODELET_ERROR("[%s::onInit] Need a 'spatial_locator' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared < dynamic_reconfigure::Server < SamplePatchesConfig > > (*pnh_);
    srv_->setCallback(bind(&SamplePatches::config_callback, this, _1, _2));

    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    // Subscribe to the clusters
    sub_clusters_filter_.subscribe(*pnh_, "clusters", max_queue_size_);

    // TODO: Surely there's some way to do this without 4 copies of this stuff
    if (approximate_sync_) {
        sync_input_clusters_a = boost::make_shared < message_filters::Synchronizer <
                                         message_filters::sync_policies::ApproximateTime <
                                         PointCloudIn, PointClusters > > > (max_queue_size_);
        // surface not enabled, connect the input-indices duo and register
        sync_input_clusters_a->connectInput(sub_input_filter_, sub_clusters_filter_);
        sync_input_clusters_a->registerCallback(bind(&SamplePatches::synchronized_input_callback, this, _1, _2));
    }
    else {
        sync_input_clusters_e_ =
                boost::make_shared < message_filters::Synchronizer < message_filters::sync_policies::ExactTime <
                PointCloudIn, PointClusters > > > (max_queue_size_);
        // surface not enabled, connect the input-indices duo and register
        sync_input_clusters_e_->connectInput(sub_input_filter_, sub_clusters_filter_);
        sync_input_clusters_e_->registerCallback(bind(&SamplePatches::synchronized_input_callback, this, _1, _2));
    }

//    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following parameters:\n"
//                   " - use_indices    : %s",
//                   getName ().c_str (),
//                   (use_indices_) ? "true" : "false");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::SamplePatches::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                                 const PointClusters::ConstPtr clusters) {
    // No subscribers, no work
    if (pub_disc_patches_.getNumSubscribers() <= 0) {
        NODELET_DEBUG("[%s::synchronized_input_callback] Input received but there are no subscribers; returning.",
                      getName().c_str());
        return;
    }

    // TODO: If cloud is given, check if it's valid

    /// DEBUG
    NODELET_DEBUG("[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
                  getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
                  cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());

    ros::WallTime start = ros::WallTime::now();

    BOOST_FOREACH(auto cluster, clusters->clusters) {
                    NODELET_DEBUG("[%s:synchronized_input_callback] Cluster with %zu points", getName().c_str(), cluster.indices.size());
                }

    NODELET_INFO_STREAM(std::setprecision(3) <<
                        "(" << (ros::WallTime::now() - start) << " sec, " << cloud->size() << " points) "
                        << "Plane fitting finished");

    // Publish a Boost shared ptr const data
    // Enforce that the TF frame and the timestamp are copied
//    patches->header = cloud->header;
//    pub_disc_patches_.publish(patches);

    NODELET_DEBUG("[%s::input_callback] Successfully published patches.", getName().c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::SamplePatches::config_callback(SamplePatchesConfig &config, uint32_t level) {
    if (spatial_locator_type_ != config.spatial_locator) {
        spatial_locator_type_ = config.spatial_locator;
        NODELET_DEBUG("[config_callback] Setting the spatial locator to type: %d.", spatial_locator_type_);
    }
    
    if (min_radius_ != config.min_radius) {
        min_radius_ = config.min_radius;
        NODELET_DEBUG("[config_callback] Setting the min radius to: %f.", min_radius_);
    }

    if (max_radius_ != config.max_radius) {
        max_radius_ = config.max_radius;
        NODELET_DEBUG("[config_callback] Setting the max radius to: %f.", max_radius_);
    }

}

PLUGINLIB_EXPORT_CLASS(surface_filters::SamplePatches, nodelet::Nodelet
)

