/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/SACSegmentAndFit.h>
#include <inttypes.h>

void surface_filters::SACSegmentAndFit::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_output_ = pnh_->advertise<NormalCloudIn>("output", max_queue_size_);
    pub_segments_ = pnh_->advertise<surface_msgs::Segment>("segments", max_queue_size_);

    int model_type_int;
    if (!pnh_->getParam("model_type", model_type_int)) {
        NODELET_ERROR ("[%s::onInit] Need a 'model_type' parameter to be set before continuing!", getName().c_str());
        return;
    }
    model_type_ = surfaces::sacModelFromConfigInt(model_type_int);
    impl_.setModelType(model_type_);
    project_.setModelType(model_type_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<SACConfig> >(*pnh_);
    srv_->setCallback(boost::bind(&SACSegmentAndFit::config_callback, this, _1, _2));

    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    // Subscribe to the clusters
    sub_clusters_filter_.subscribe(*pnh_, "clusters", max_queue_size_);

    // TODO: Surely there's some way to do this without 4 copies of this stuff
    if (use_normals_) {
        // Subscribe to the normals
        sub_normals_filter_.subscribe(*pnh_, "normals", max_queue_size_);

        if (approximate_sync_) {
            sync_input_normals_clusters_a_ = boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, NormalCloudIn, PointClusters> >(
                    max_queue_size_);
            sync_input_normals_clusters_a_->connectInput(sub_input_filter_, sub_normals_filter_, sub_clusters_filter_);
            sync_input_normals_clusters_a_->registerCallback(
                    bind(&SACSegmentAndFit::synchronized_input_callback, this, _1, _2, _3));
        } else {
            sync_input_normals_clusters_e_ = boost::make_shared<ExactTimeSynchronizer<PointCloudIn, NormalCloudIn, PointClusters> >(
                    max_queue_size_);
            sync_input_normals_clusters_e_->connectInput(sub_input_filter_, sub_normals_filter_, sub_clusters_filter_);
            sync_input_normals_clusters_e_->registerCallback(
                    bind(&SACSegmentAndFit::synchronized_input_callback, this, _1, _2, _3));
        }
    } else {
        if (approximate_sync_) {
            sync_input_clusters_a_ = boost::make_shared<ApproximateTimeSynchronizer<PointCloudIn, PointClusters> >(
                    max_queue_size_);
            sync_input_clusters_a_->connectInput(sub_input_filter_, sub_clusters_filter_);
            sync_input_clusters_a_->registerCallback(
                    bind(&SACSegmentAndFit::synchronized_input_callback, this, _1, NormalCloudIn::ConstPtr(), _2));
        } else {
            sync_input_clusters_e_ = boost::make_shared<ExactTimeSynchronizer<PointCloudIn, PointClusters> >(
                    max_queue_size_);
            sync_input_clusters_e_->connectInput(sub_input_filter_, sub_clusters_filter_);
            sync_input_clusters_e_->registerCallback(
                    bind(&SACSegmentAndFit::synchronized_input_callback, this, _1, NormalCloudIn::ConstPtr(), _2));
        }
    }

    NODELET_DEBUG("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                          " - model_type    : %d",
                  getName().c_str(),
                  model_type_);
}

void surface_filters::SACSegmentAndFit::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                               const NormalCloudIn::ConstPtr &normals_ros,
                                                               const PointClusters::ConstPtr &input_clusters) {
    // No subscribers, no work
    if (pub_output_.getNumSubscribers() <= 0 && pub_segments_.getNumSubscribers() <= 0) {
        NODELET_DEBUG("[%s::synchronized_input_callback] Input received but there are no subscribers; returning.",
                      getName().c_str());
        return;
    }

    // TODO: Check validity of input

    if (normals_ros != NULL) {
        NODELET_DEBUG("[%s::synchronized_input_callback]\n"
                              "                                 - PointCloud with %d data points(%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - Normals PointCloud with %d data points(%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - PointClusters with %zu clusters, stamp %f, and frame %s on topic %s received.",
                      getName().c_str(),

                      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
                      fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("input").c_str(),

                      normals_ros->width * normals_ros->height, pcl::getFieldsList(*normals_ros).c_str(),
                      fromPCL(normals_ros->header).stamp.toSec(), normals_ros->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("normals").c_str(),

                      input_clusters->clusters.size(),
                      fromPCL(input_clusters->header).stamp.toSec(), input_clusters->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("clusters").c_str());
    } else {
        NODELET_DEBUG("[%s::synchronized_input_callback]\n"
                              "                                 - PointCloud with %d data points(%s), stamp %f, and frame %s on topic %s received.\n"
                              "                                 - PointClusters with %zu clusters, stamp %f, and frame %s on topic %s received.",
                      getName().c_str(),

                      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
                      fromPCL(cloud->header).stamp.toSec(), cloud->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("input").c_str(),

                      input_clusters->clusters.size(),
                      fromPCL(input_clusters->header).stamp.toSec(), input_clusters->header.frame_id.c_str(),
                      getMTPrivateNodeHandle().resolveName("clusters").c_str());

    }


    bool should_populate_output_cloud = pub_output_.getNumSubscribers() > 0;

    // Output points have the same type as the input (enforced by the pcl_nodelet class)
    PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();

    impl_.setInputCloud(cloud);
    project_.setInputCloud(cloud);

    ros::WallTime start = ros::WallTime::now();

    NODELET_DEBUG("[%s::synchronized_input_callback] Performing SAC segmentation and fitting with should_output_cloud = %d",
                  getName().c_str(), should_populate_output_cloud);

    for (PointIndices input_cluster : input_clusters->clusters) {
        PointIndices::Ptr remaining_cluster = boost::make_shared<PointIndices>(input_cluster);
        PointIndices::Ptr remaining_temp;
        bool first_run = true;
        do {
            PointIndices::Ptr indices = boost::make_shared<PointIndices>();
            Segment::Ptr segment = boost::make_shared<Segment>();

            // Limit SAC to only the points in this cluster
            impl_.setIndices(remaining_cluster);

            // Do the segmentation
            impl_.segment(*indices, segment->model);

            if (indices->indices.size() < min_points_) break;

            segment->inliers = PointCloudIn(*cloud, indices->indices);

            if (should_populate_output_cloud) {
                uint8_t r = (uint8_t) (rand() % 256), g = (uint8_t) (rand() % 256), b = (uint8_t) (rand() % 256);

                size_t outputPointsSize = output->points.size();
                size_t projectedPointsSize = segment->inliers.points.size();
                output->points.resize(outputPointsSize + projectedPointsSize);

                for (size_t i = 0; i < projectedPointsSize; i++) {
                    output->points[outputPointsSize + i].x = segment->inliers.points[i].x;
                    output->points[outputPointsSize + i].y = segment->inliers.points[i].y;
                    output->points[outputPointsSize + i].z = segment->inliers.points[i].z;
                    output->points[outputPointsSize + i].r = r;
                    output->points[outputPointsSize + i].g = g;
                    output->points[outputPointsSize + i].b = b;
                }
            }

            // Only sort & make a copy if necessary
            if (first_run) {
                remaining_temp = boost::make_shared<PointIndices>();
                remaining_temp->header = remaining_cluster->header;

                std::sort(remaining_cluster->indices.begin(), remaining_cluster->indices.end());
                first_run = false;
            } else {
                remaining_temp->indices.clear();
            }

            // The inliers have no guarantee of sorted-ness
            std::sort(indices->indices.begin(), indices->indices.end());

            // Copies everything in remaining_src EXCEPT the contents of inliners into remaining_temp
            std::set_difference(remaining_cluster->indices.begin(), remaining_cluster->indices.end(),
                                indices->indices.begin(), indices->indices.end(),
                                std::back_inserter(remaining_temp->indices));

            remaining_cluster.swap(remaining_temp);

            segment->header = cloud->header;
            segment->surface_id = Segment::NEW_SURFACE;
            pub_segments_.publish(segment);

            NODELET_DEBUG("[%s] Published segment with %zu points at time %" PRIu64,
                          getName().c_str(), segment->inliers.size(), segment->header.stamp);

        } while (remaining_cluster->indices.size() >= min_points_);
    }

    NODELET_INFO_STREAM("(" << std::setprecision(3) << (ros::WallTime::now() - start) << " sec, " << cloud->size() << " points) "
                        << "Plane fitting finished");

    // Publish a Boost shared ptr const data
    // Enforce that the TF frame and the timestamp are copied
    output->header = cloud->header;
    pub_output_.publish(output);

    NODELET_DEBUG("[%s::synchronized_input_callback] Successfully published output and clusters.", getName().c_str());
}

void surface_filters::SACSegmentAndFit::config_callback(SACConfig &config, uint32_t level __attribute((unused))) {
    if (use_normals_ != config.use_normals) {
        use_normals_ = config.use_normals;
        NODELET_DEBUG ("[config_callback] Setting use normals: %d.", use_normals_);
    }

    if (model_type_ != surfaces::sacModelFromConfigInt(config.model_type)) {
        model_type_ = surfaces::sacModelFromConfigInt(config.model_type);
        impl_.setModelType(model_type_);
        project_.setModelType(model_type_);
        NODELET_DEBUG ("[config_callback] Setting the model type to: %d.", model_type_);
    }

    if (method_type_ != config.method_type) {
        impl_.setMethodType(config.method_type);
        method_type_ = config.method_type;
        NODELET_DEBUG ("[config_callback] Setting the method type to: %d.", method_type_);
    }

    if (dist_threshold_ != config.distance_threshold) {
        impl_.setDistanceThreshold(config.distance_threshold);
        dist_threshold_ = config.distance_threshold;
        NODELET_DEBUG ("[config_callback] Setting distance threshold to: %f.", dist_threshold_);
    }

    if (max_iterations_ != config.max_iterations) {
        impl_.setMaxIterations(config.max_iterations);
        max_iterations_ = config.max_iterations;
        NODELET_DEBUG ("[config_callback] Setting the max iterations to: %d.", max_iterations_);
    }

    if (probability_ != config.probability) {
        impl_.setProbability(config.probability);
        probability_ = config.probability;
        NODELET_DEBUG ("[config_callback] Setting the probability to: %f.", probability_);
    }

    if (optimize_coefficients_ != config.optimize_coefficients) {
        impl_.setOptimizeCoefficients(config.optimize_coefficients);
        optimize_coefficients_ = config.optimize_coefficients;
        NODELET_DEBUG ("[config_callback] Setting optimize coefficients to: %d.", optimize_coefficients_);
    }

    if (radius_min_ != config.radius_min) {
        impl_.setRadiusLimits(config.radius_min, radius_max_);
        radius_min_ = config.radius_min;
        NODELET_DEBUG ("[config_callback] Setting the min radius to: %f.", radius_min_);
    }

    if (radius_max_ != config.radius_max) {
        impl_.setRadiusLimits(config.radius_max, radius_max_);
        radius_max_ = config.radius_max;
        NODELET_DEBUG ("[config_callback] Setting the max radius to: %f.", radius_max_);
    }

    if (epsilon_angle_ != config.epsilon_angle) {
        impl_.setEpsAngle(config.epsilon_angle);
        epsilon_angle_ = config.epsilon_angle;
        NODELET_DEBUG ("[config_callback] Setting the epsilon angle to: %f.", epsilon_angle_);
    }

    if (min_points_ != (unsigned int) config.min_points) {
        min_points_ = (unsigned int) config.min_points;
        NODELET_DEBUG ("[config_callback] Setting the min points to: %ud.", min_points_);
    }
}

PLUGINLIB_EXPORT_CLASS(surface_filters::SACSegmentAndFit, nodelet::Nodelet)
