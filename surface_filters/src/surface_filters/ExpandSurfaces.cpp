//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ExpandSurfaces.h>

//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::ExpandSurfaces::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_replace_surface_ = pnh_->advertise<SurfaceStamped>("replace_surface", max_queue_size_);
    pub_remaining_indices_ = pnh_->advertise<PointIndices>("removed_indices", max_queue_size_);

//    if (!pnh_->getParam("resolution", resolution_)) {
//        NODELET_ERROR("[%s::onInit] Need a 'resolution' parameter to be set before continuing!",
//                      getName().c_str());
//        return;
//    }

    // Enable the dynamic reconfigure service
//    srv_ = boost::make_shared<dynamic_reconfigure::Server<ChangeDetectionConfig> >(*pnh_);
//    srv_->setCallback(boost::bind(&ChangeDetection::config_callback, this, _1, _2));

    sub_input_filter_.subscribe(*pnh_, "input", max_queue_size_);
    sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);

    if (approximate_sync_) {
        sync_input_indices_a_ = boost::make_shared<ApproxTimeSynchronizer>(ApproxPolicy(max_queue_size_),
                                                                           sub_input_filter_, sub_indices_filter_);
        sync_input_indices_a_->registerCallback(bind(&ExpandSurfaces::synchronized_input_callback, this, _1, _2));
    } else {
        sync_input_indices_e_ = boost::make_shared<ExactTimeSynchronizer>(ExactPolicy(max_queue_size_),
                                                                          sub_input_filter_, sub_indices_filter_);
        sync_input_indices_e_->registerCallback(bind(&ExpandSurfaces::synchronized_input_callback, this, _1, _2));
    }

    sub_surfaces_.subscribe(*pnh_, "surfaces", max_queue_size_);
    surfaces_cache_ = boost::make_shared<message_filters::Cache<Surfaces> >(sub_surfaces_, 1);
    
//    sub_surface_clouds_.subscribe(*pnh_, "surface_clouds", max_queue_size_);
//    surface_clouds_cache_ = boost::make_shared<message_filters::Cache<SurfaceClouds> >(sub_surface_clouds_, 1);

    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following subscriptions:\n"
                           " - input    : %s\n"
                           " - indices    : %s\n"
                           " - surfaces    : %s\n",
                   getName().c_str(),
                   getMTPrivateNodeHandle().resolveName("input").c_str(),
                   getMTPrivateNodeHandle().resolveName("indices").c_str(),
                   getMTPrivateNodeHandle().resolveName("surfaces").c_str());

}


Eigen::Affine3f tf_from_plane_model(pcl_msgs::ModelCoefficients &plane) {
    Eigen::Vector3f new_normal(plane.values[0], plane.values[1], plane.values[2]);
    Eigen::Vector3f old_normal(0, 0, 1);

    Eigen::Vector3f v = old_normal.cross(new_normal);
    float s2 = v.squaredNorm();
    float c = old_normal.dot(new_normal);
    Eigen::Matrix3f v_cross;
    v_cross << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;

    Eigen::Matrix3f rot = Eigen::Matrix3f::Identity() + v_cross + v_cross * v_cross * (1 - c) / s2;
    Eigen::Affine3f arot(rot);

    // Create a transform where the rotation component is given by the rotation axis as the normal vector (a, b, c)
    // and some arbitrary angle about that axis and the translation component is -d in the z direction after
    // that rotation (not the original z direction, which is how transforms are usually defined).
    return arot * Eigen::Translation3f(0, 0, -plane.values[3]);

}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::ExpandSurfaces::synchronized_input_callback(const PointCloudIn::ConstPtr &cloud,
                                                                  const pcl_msgs::PointIndices::ConstPtr &indices) {
    // No subscribers, no work
//    if (pub_replace_surface_.getNumSubscribers() <= 0) {
//        NODELET_DEBUG("[%s::synchronized_input_callback] Input received but there are no subscribers; returning.",
//                      getName().c_str());
//        return;
//    }

    // TODO: If cloud is given, check if it's valid

    if (cloud->width * cloud->height == 0 || indices->indices.size() == 0) {
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

    ros::WallTime start = ros::WallTime::now();

    Surfaces::ConstPtr surfaces = surfaces_cache_->getElemBeforeTime(ros::Time::now());

    if (surfaces == NULL) {
//        NODELET_DEBUG("[%s::input_callback] No surfaces yet... returning.", getName().c_str());
        return;
    }

    crop_.setInputCloud(cloud);
    // Bounds are infinite in X and Y and finite in Z
    crop_.setMax(Eigen::Vector4f(std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<float>::infinity(),
                                 perpendicular_dist_threshold_, 0));
    crop_.setMin(Eigen::Vector4f(-std::numeric_limits<float>::infinity(),
                                 -std::numeric_limits<float>::infinity(),
                                 -perpendicular_dist_threshold_, 0));


    // Create a mutex wrapper but don't immediately lock
    std::unique_lock<std::mutex> hull_lock(hull_mutex_, std::defer_lock);

    boost::shared_ptr<std::vector<int> > unprocessed_point_indices = boost::make_shared<std::vector<int> >(indices->indices);
    // unprocessed_point_indices must be sorted to work with the set_difference function at the bottom of the while
    std::sort(unprocessed_point_indices->begin(), unprocessed_point_indices->end());
    boost::shared_ptr<std::set<int> > within_radius_indices = boost::make_shared<std::set<int> >();
    boost::shared_ptr<std::vector<int> > within_radius_inplane_indices = boost::make_shared<std::vector<int> >();
    boost::shared_ptr<std::vector<int> > tmp_indices = boost::make_shared<std::vector<int> >();
    boost::shared_ptr<std::vector<std::vector<int> > > tmp_multi_indices = boost::make_shared<std::vector<std::vector<int> > >();
    boost::shared_ptr<std::vector<std::vector<float> > > tmp_sqrdistances = boost::make_shared<std::vector<std::vector<float>>>();
    for (Surface old_surface : surfaces->surfaces) {
        // Make a PCL copy of the important data in the Surface
        pcl::PolygonMesh::Ptr concave_hull = boost::make_shared<pcl::PolygonMesh>();
        pcl_conversions::toPCL(old_surface.concave_hull, *concave_hull);

        PointCloudIn::Ptr edge_points = boost::make_shared<PointCloudIn>();
        pcl::fromPCLPointCloud2(concave_hull->cloud, *edge_points);

        PointCloudIn::Ptr inliers = boost::make_shared<PointCloudIn>();
        pcl::fromROSMsg(old_surface.inliers, *inliers);

        bool surface_changed = false;
        while (unprocessed_point_indices->size() > 0) { // Breaks inside
//            NODELET_DEBUG("[%s::input_callback] ExpandSurface (re-)considering surface %u",
//                          getName().c_str(), old_surface.id);


            //
            // FILTER TO POINTS NEAR SURFACE BOUNDARY
            //
            search_.setInputCloud(cloud, unprocessed_point_indices);

            within_radius_indices->clear();
            tmp_multi_indices->clear();
            tmp_indices->clear();  // An empty argument for indices means to search all points
            // radiusSearch([in] PointCloud cloud, [in] vector[int] indices, [in] double radius,
            //              [out] vector[vector[int]] indices, [out] vector[vector[float]] sqrdistances)
            search_.radiusSearch(*edge_points, *tmp_indices, parallel_dist_threshold_, *tmp_multi_indices,
                                 *tmp_sqrdistances);
            for(std::vector<int>& ind : *tmp_multi_indices) {
                within_radius_indices->insert(ind.begin(), ind.end());
            }

            if (within_radius_indices->size() == 0) break;

            //
            // FILTER TO POINTS ON THE SURFACE
            //
            crop_.setIndices(boost::make_shared<std::vector<int> > (within_radius_indices->begin(),
                                                                    within_radius_indices->end()));

            Eigen::Affine3f plane_tf = tf_from_plane_model(old_surface.model);
            const Eigen::Transform<float, 3, 2, 0>::TranslationPart &trans = plane_tf.translation();
            crop_.setTranslation(Eigen::Vector3f(trans.x(), trans.y(), trans.z()));
            Eigen::Vector3f rotation;
            pcl::getEulerAngles(plane_tf, rotation(0), rotation(1), rotation(2));
            crop_.setRotation(rotation);

            within_radius_inplane_indices->clear();
            crop_.filter(*within_radius_inplane_indices);

            if (within_radius_inplane_indices->size() == 0) break;

            //
            // ADD NEW POINTS TO SURFACE
            //
            auto prev_size = inliers->size();

            surface_changed = true;
            auto new_points_cloud = PointCloudIn(*cloud, *within_radius_inplane_indices);
            *inliers += new_points_cloud;
            *edge_points += new_points_cloud;

//            NODELET_INFO_STREAM("Surface expanded from " << prev_size << " points to " << inliers->size() << " points");
            
            //
            // REMOVE THOSE POINTS FROM THE INPUT
            //
            auto prev_remaining = unprocessed_point_indices->size();
            tmp_indices->clear();
            std::sort(within_radius_inplane_indices->begin(), within_radius_inplane_indices->end());
            // Set difference requirements/invariants: both inputs must be sorted and the output will be sorted
            std::set_difference(unprocessed_point_indices->begin(), unprocessed_point_indices->end(),
                                within_radius_inplane_indices->begin(), within_radius_inplane_indices->end(),
                                std::inserter(*tmp_indices, tmp_indices->begin()));
            tmp_indices.swap(unprocessed_point_indices);
            auto new_remaining = unprocessed_point_indices->size();
//            NODELET_INFO_STREAM("Unprocessed indices reduced from " << prev_remaining << " indices to " << new_remaining << " indices");
        }

        if (surface_changed) {
            // Concave hull is not thread save
            hull_lock.lock();
            hull_.setAlpha(concave_hull_alpha_);
            hull_.setInputCloud(inliers);
            hull_.reconstruct(*concave_hull);
            hull_lock.unlock();

            SurfaceStamped new_surface;
            new_surface.header = indices->header;
            new_surface.surface.id = old_surface.id;
            new_surface.surface.color = old_surface.color;
            new_surface.surface.model = old_surface.model;
            pcl_conversions::fromPCL(*concave_hull, new_surface.surface.concave_hull);
            pcl::toROSMsg(*inliers, new_surface.surface.inliers);

            pub_replace_surface_.publish(new_surface);

//            NODELET_DEBUG("[%s::input_callback] ExpandSurface expanded surface %u from %z to %z points.",
//                          getName().c_str(), new_surface.surface.id, old_surface.inliers.height * old_surface.inliers.height,
//                          new_surface.surface.inliers.height * new_surface.surface.inliers.width);
        }
    }

    // Always publish remaining indices
    PointIndices::Ptr new_indices = boost::make_shared<PointIndices>();
    new_indices->header = cloud->header;
    new_indices->indices = *unprocessed_point_indices;
    pub_remaining_indices_.publish(new_indices);
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
