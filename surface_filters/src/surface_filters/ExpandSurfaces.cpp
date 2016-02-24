//
// Created by will on 2/17/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_filters/ExpandSurfaces.h>

//////////////////////////////////////////////////////////////////////////////////////////////

void surface_filters::ExpandSurfaces::onInit() {
    pcl_ros::PCLNodelet::onInit();

    pub_replace_surface_ = pnh_->advertise<PointCloudOut>("replace_surface", max_queue_size_);
    pub_removed_indices_ = pnh_->advertise<PointCloudOut>("removed_indices", max_queue_size_);

//    if (!pnh_->getParam("resolution", resolution_)) {
//        NODELET_ERROR("[%s::onInit] Need a 'resolution' parameter to be set before continuing!",
//                      getName().c_str());
//        return;
//    }

    // Enable the dynamic reconfigure service
//    srv_ = boost::make_shared<dynamic_reconfigure::Server<ChangeDetectionConfig> >(*pnh_);
//    srv_->setCallback(boost::bind(&ChangeDetection::config_callback, this, _1, _2));

    pnh_->getParam("use_indices", use_indices_);
    if (use_indices_) {
        // If using indices, subscribe to the input and indices using a filter
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
    } else {
        // If not using indices, subscribe to the cloud directly
        sub_input_ = pnh_->subscribe<PointCloudIn>("input", max_queue_size_,
                                                   bind(&ExpandSurfaces::synchronized_input_callback, this, _1, PointIndicesConstPtr()));
    }


    sub_surfaces_.subscribe(*pnh_, "surfaces", max_queue_size_);
    surfaces_cache_ = boost::make_shared<message_filters::Cache<Surfaces> >(sub_surfaces_, 1);
}


Eigen::Affine3f tf_from_plane_model(pcl::ModelCoefficients &plane) {
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

    /// DEBUG
    NODELET_DEBUG("[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.",
                  getName().c_str(), cloud->width * cloud->height, fromPCL(cloud->header).stamp.toSec(),
                  cloud->header.frame_id.c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());

    ros::WallTime start = ros::WallTime::now();

    Surfaces::ConstPtr surfaces = surfaces_cache_->getElemBeforeTime(ros::Time::now());

    crop_.setInputCloud(cloud);
    // Bounds are infinite in X and Y and finite in Z
    crop_.setMax(Eigen::Vector4f(std::numeric_limits<float>::infinity(),
                                 std::numeric_limits<float>::infinity(),
                                 perpendicular_dist_threshold_, 0));
    crop_.setMin(Eigen::Vector4f(-std::numeric_limits<float>::infinity(),
                                 -std::numeric_limits<float>::infinity(),
                                 -perpendicular_dist_threshold_, 0));

    hull_.setAlpha(concave_hull_alpha_);

    boost::shared_ptr<std::vector<int> > all_removed_indices = boost::make_shared<std::vector<int> >();
    boost::shared_ptr<std::vector<int> > filter_indices = boost::make_shared<std::vector<int> >(indices->indices);
    boost::shared_ptr<std::vector<int> > tmp_indices = boost::make_shared<std::vector<int> >(indices->indices);
    boost::shared_ptr<std::vector<std::vector<int> > > tmp_multi_indices = boost::make_shared<std::vector<std::vector<int> > >();
    boost::shared_ptr<std::vector<std::vector<float> > > tmp_sqrdistances = boost::make_shared<std::vector<std::vector<float>>>();
    for (Surface old_surface : surfaces->surfaces) {
        // Make a copy of the surface that we can mutate and publish
        Surface::Ptr surface = boost::make_shared<Surface>(old_surface);

        bool surface_changed = false;
//        while (true) { // Breaks inside
//            NODELET_DEBUG("[%s::input_callback] ExpandSurface (re-)considering surface %u",
//                          getName().c_str(), surface->id);

//            PointCloudIn concave_hull_cloud;
//            pcl::fromROSMsg(surface->concave_hull.cloud, concave_hull_cloud);

//            // TODO: Cache this somehow [make it part of the message?]
//            std::vector<int> edge_pts;
//            BOOST_FOREACH(pcl_msgs::Vertices vertices, surface->concave_hull.polygons) {
//                            edge_pts.insert(edge_pts.end(), vertices.vertices.begin(),
//                                            vertices.vertices.end());
//                        }
//            //
//            // FILTER TO POINTS NEAR SURFACE BOUNDARY
//            //
//            search_.setInputCloud(cloud, filter_indices);

//            tmp_indices->clear();
//            tmp_multi_indices->clear();
//            // radiusSearch([in] PointCloud cloud, [in] vector[int] indices, [in] double radius,
//            //              [out] vector[vector[int]] indices, [out] vector[vector[float]] sqrdistances)
//            search_.radiusSearch(concave_hull_cloud, edge_pts, parallel_dist_threshold_, *tmp_multi_indices,
//                                 *tmp_sqrdistances);
//            BOOST_FOREACH(std::vector<int> ind, *tmp_multi_indices) {
//                            tmp_indices->insert(tmp_indices->end(), ind.begin(), ind.end());
//                        }

//            filter_indices.swap(tmp_indices);

//            if (filter_indices->size() == 0) break;

//            //
//            // FILTER TO POINTS ON THE SURFACE
//            //
//            crop_.setIndices(filter_indices);

//            Eigen::Affine3f plane_tf = tf_from_plane_model(surface->model);
//            const Eigen::Transform<float, 3, 2, 0>::TranslationPart &trans = plane_tf.translation();
//            crop_.setTranslation(Eigen::Vector3f(trans.x(), trans.y(), trans.z()));
////                        crop_.setRotation();

//            tmp_indices->clear();
//            crop_.filter(*tmp_indices);
//            filter_indices.swap(tmp_indices);

//            if (filter_indices->size() == 0) break;

//            //
//            // ADD NEW POINTS TO SURFACE
//            //
//            surface_changed = true;
//            surface->inliers += PointCloudIn(*cloud, *filter_indices);

//            // hull only takes a shared_ptr :( [this makes a copy]
//            PointCloudIn::Ptr inliers_ptr = boost::make_shared<PointCloudIn>(surface->inliers);
//            // hull wants a point cloud
//            PointCloudIn::Ptr tmp_cloud = boost::make_shared<PointCloudIn>();
//            hull_.setInputCloud(inliers_ptr);
//            hull_.reconstruct(*tmp_cloud, surface->convex_hull.polygons);

//            //
//            // REMOVE THOSE POINTS FROM THE INPUT
//            //
//            all_removed_indices->insert(all_removed_indices->end(), filter_indices->begin(), filter_indices->end());
//        }

//        if (surface_changed) {
//            pub_replace_surface_.publish(surface);

//            NODELET_DEBUG("[%s::input_callback] ExpandSurface expanded surface %u from %zu to %zu points.",
//                          getName().c_str(), surface->id, old_surface.inliers.size(), surface->inliers.size());

//        }
    }

    // Always publish remaining indices
    PointIndices::Ptr new_indices = boost::make_shared<PointIndices>();
    new_indices->header = cloud->header;
    new_indices->indices = *all_removed_indices;
    pub_removed_indices_.publish(new_indices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//void surface_filters::ExpandSurfaces::config_callback(ChangeDetectionConfig &config, uint32_t level) {
//    if (resolution_ != config.resolution) {
//        resolution_ = config.resolution;
//        impl_ = ChangeDetector(resolution_);
//        NODELET_DEBUG("[config_callback] Setting the resolution to: %f. Note that changing the resolution requires restarting change detection", resolution_);
//    }
//
//    if (min_points_in_leaf_ != config.min_points_per_leaf) {
//        min_points_in_leaf_ = config.min_points_per_leaf;
//        NODELET_DEBUG("[config_callback] Setting the max radius to: %d.", min_points_in_leaf_);
//    }
//}
