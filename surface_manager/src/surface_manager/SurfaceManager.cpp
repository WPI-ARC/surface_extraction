//
// Created by will on 2/15/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_manager/SurfaceManager.h>

void surface_manager::SurfaceManager::onInit() {
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    private_nh.param("max_queue_size", max_queue_size_, 3);
    private_nh.param("publish_interval", publish_interval_, static_cast<float>(1.0));
    private_nh.param("target_frame", target_frame_, static_cast<std::string>("/world"));

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<SurfaceManagerConfig> >(private_nh);
    srv_->setCallback(boost::bind(&SurfaceManager::config_callback, this, _1, _2));

    // Advertise a topic called "surfaces" with a queue size of max_queue_size and latched=true
    surfaces_pub_ = private_nh.advertise<Surfaces>("surfaces", static_cast<uint32_t>(max_queue_size_), true);
    surface_meshes_pub_ = private_nh.advertise<SurfaceMeshes>("surface_meshes", static_cast<uint32_t>(max_queue_size_), true);
    output_pub_ = private_nh.advertise<PointCloudOut>("output", static_cast<uint32_t>(max_queue_size_));
    perimeter_pub_ = private_nh.advertise<PointCloudOut>("perimeter", static_cast<uint32_t>(max_queue_size_));
    visualization_pub_ = private_nh.advertise<vis::MarkerArray>("surfaces_visualization", static_cast<uint32_t>(max_queue_size_));

    // Subscribe to inputs (for adding surfaces)
    new_surface_sub_.subscribe(private_nh, "add_surface", static_cast<uint32_t>(max_queue_size_));
    new_surface_mesh_sub_.subscribe(private_nh, "add_surface_mesh", static_cast<uint32_t>(max_queue_size_));
    updated_surface_sub_.subscribe(private_nh, "updated_surface", static_cast<uint32_t>(max_queue_size_));
    updated_surface_mesh_sub_.subscribe(private_nh, "updated_surface_mesh", static_cast<uint32_t>(max_queue_size_));

    new_surface_synchronizer_ = boost::make_shared<SurfaceSynchronizer>(max_queue_size_);
    new_surface_synchronizer_->connectInput(new_surface_sub_, new_surface_mesh_sub_);
    new_surface_synchronizer_->registerCallback(bind(&SurfaceManager::add_surface, this, _1, _2));

    updated_surface_synchronizer_ = boost::make_shared<SurfaceSynchronizer>(max_queue_size_);
    updated_surface_synchronizer_->connectInput(updated_surface_sub_, updated_surface_mesh_sub_);
    updated_surface_synchronizer_->registerCallback(bind(&SurfaceManager::replace_surface, this, _1, _2));

    // Start publisher timer
    publish_timer_ = private_nh.createTimer(ros::Duration(publish_interval_), bind(&SurfaceManager::publish, this, _1));

    NODELET_DEBUG("[%s::onInit] SurfaceManager started with max_queue_size = %d\n", getName().c_str(), max_queue_size_);
}

void surface_manager::SurfaceManager::add_surface(const SurfaceStamped::ConstPtr surface,
                                                  const SurfaceMeshStamped::ConstPtr mesh) {
    auto hull_size = surface->surface.concave_hull.cloud.width * surface->surface.concave_hull.cloud.height;
    NODELET_DEBUG_STREAM("New Surface received with " << surface->surface.inliers.size() << " points, " << hull_size <<
                                 " vertices, " << mesh->surface_mesh.surface_mesh.triangles.size() << " triangles");

    auto pos = std::lower_bound(surfaces_.begin(), surfaces_.end(), surface->surface, surface_lower_bound_comparator());
    auto &new_surface = surfaces_.insert(pos, {surface->surface, mesh->surface_mesh})->first;
    new_surface.id = next_surface_id_++;
    new_surface.color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    new_surface.color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    new_surface.color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    new_surface.color.a = 1;
}

void surface_manager::SurfaceManager::replace_surface(const SurfaceStamped::ConstPtr surface,
                                                      const SurfaceMeshStamped::ConstPtr mesh) {
    NODELET_DEBUG("Replacement for surface %u received", surface->surface.id);
    auto prev_pos = std::find_if(surfaces_.begin(), surfaces_.end(), [&surface](SurfaceMeshPair &test_surface) {
        return test_surface.first.id == surface->surface.id;
    });

    auto pos = std::lower_bound(surfaces_.begin(), surfaces_.end(), surface->surface, surface_lower_bound_comparator());

    // Replace surface but maintain color
    auto color = prev_pos->first.color;
    *prev_pos = {surface->surface, mesh->surface_mesh};
    prev_pos->first.color = color;

    if (std::distance(prev_pos, pos) > 0) {
        std::rotate(prev_pos, prev_pos+1, pos);
    } else if (std::distance(prev_pos, pos) < 0) {
        using reverse = std::vector<SurfaceMeshPair>::reverse_iterator;
        std::rotate(reverse(prev_pos)-1, reverse(prev_pos), reverse(pos));
    }

    NODELET_DEBUG_STREAM("Old version of surface found at " << std::distance(surfaces_.begin(), prev_pos) <<
                         " and moved to " << std::distance(surfaces_.begin(), pos) <<
                         " (length of array is " << surfaces_.size() << ")");
}

void surface_manager::SurfaceManager::publish(const ros::TimerEvent &event) const {
    if (surfaces_.size() < 1) return;

    this->publish_surfaces_mesh_pairs();
    this->publish_inliers();
    this->publish_perimeter_points();
    this->publish_perimeter_lines();
    this->publish_mesh_triangles();

    NODELET_DEBUG("SurfaceManager published surfaces, meshes, and visualizations");
}

void surface_manager::SurfaceManager::publish_mesh_triangles() const {
    if (this->visualization_pub_.getNumSubscribers() > 0) {
        vis::MarkerArray markers;

        for (const SurfaceMeshPair &surface_mesh_pair : this->surfaces_) {
            const Surface &surface = surface_mesh_pair.first;
            const shape_msgs::Mesh &trimesh = surface_mesh_pair.second.surface_mesh;

            vis::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = this->target_frame_;
            marker.ns = "surface_triangles";
            marker.id = surface.id;
            marker.type = vis::Marker::TRIANGLE_LIST;
            marker.action = vis::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            marker.color = surface.color;

            marker.points.reserve(trimesh.triangles.size() * 3);
            for (const shape_msgs::MeshTriangle &triangle : trimesh.triangles) {
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[0]]);
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[1]]);
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[2]]);
            }

            markers.markers.push_back(marker);
        }

        this->visualization_pub_.publish(markers);
    }
}

void surface_manager::SurfaceManager::publish_perimeter_lines() const {
    if (this->visualization_pub_.getNumSubscribers() > 0) {
        vis::MarkerArray all_markers;

        for (const SurfaceMeshPair &surface_pair : this->surfaces_) {
            const Surface &surface = surface_pair.first;

            PointCloudIn perimeter_cloud;
            fromPCLPointCloud2(surface.concave_hull.cloud, perimeter_cloud);

            vis::Marker perimeter;
            perimeter.header.stamp = ros::Time::now();
            perimeter.header.frame_id = this->target_frame_;
            perimeter.ns = "surface_perimeter";
            perimeter.id = surface.id;
            perimeter.type = vis::Marker::LINE_LIST;
            perimeter.action = vis::Marker::MODIFY;
            // perimeter.pose not needed
            perimeter.scale.x = 0.01; // scale.x controls the width of the line segments
            perimeter.color = surface.color;

            for (const pcl::Vertices &polygon : surface.concave_hull.polygons) {
                for (size_t i = 0; i < polygon.vertices.size(); i++) {
                    geometry_msgs::Point pointA, pointB;

                    const pcl::PointXYZ &pt_i = perimeter_cloud.points[polygon.vertices[i]];
                    pointA.x = pt_i.x; pointA.y = pt_i.y; pointA.z = pt_i.z;

                    // Connect each point to the next, and the last point back to the first
                    unsigned long next_i = (i + 1) % polygon.vertices.size();

                    const pcl::PointXYZ &pt_nexti = perimeter_cloud.points[polygon.vertices[next_i]];
                    pointB.x = pt_nexti.x; pointB.y = pt_nexti.y; pointB.z = pt_nexti.z;

                    perimeter.points.push_back(pointA);
                    perimeter.points.push_back(pointB);
                }
            }

            all_markers.markers.push_back(perimeter);
        }

        this->visualization_pub_.publish(all_markers);
    }
}

void surface_manager::SurfaceManager::publish_perimeter_points() const {
    if (this->perimeter_pub_.getNumSubscribers() > 0) {
        PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();

        for (const SurfaceMeshPair &surface_pair : this->surfaces_) {
            const Surface &surface = surface_pair.first;

            PointCloudIn perimeter_cloud;
            fromPCLPointCloud2(surface.concave_hull.cloud, perimeter_cloud);

            output->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            output->header.frame_id = this->target_frame_;
            output->points.reserve(output->size() + surface.inliers.size());

            uint8_t r = static_cast<uint8_t>(surface.color.r * 255);
            uint8_t g = static_cast<uint8_t>(surface.color.g * 255);
            uint8_t b = static_cast<uint8_t>(surface.color.b * 255);
            for (const PointIn &pt : perimeter_cloud.points) {
                output->points.emplace_back(r, g, b);
                output->points.back().x = pt.x;
                output->points.back().y = pt.y;
                output->points.back().z = pt.z;
            }
        }

        this->perimeter_pub_.publish(output);
    }
}

void surface_manager::SurfaceManager::publish_inliers() const {
    if (this->output_pub_.getNumSubscribers() > 0) {
        PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();

        for (const SurfaceMeshPair &surface_pair : this->surfaces_) {
            const Surface &surface = surface_pair.first;
            output->header.stamp = pcl_conversions::toPCL(ros::Time::now());
            output->header.frame_id = this->target_frame_;
            output->points.reserve(output->size() + surface.inliers.size());

            uint8_t r = static_cast<uint8_t>(surface.color.r * 255);
            uint8_t g = static_cast<uint8_t>(surface.color.g * 255);
            uint8_t b = static_cast<uint8_t>(surface.color.b * 255);
            for (const PointIn &pt : surface.inliers.points) {
                output->points.emplace_back(r, g, b);
                output->points.back().x = pt.x;
                output->points.back().y = pt.y;
                output->points.back().z = pt.z;
            }
        }

        this->output_pub_.publish(output);
    }
}

void surface_manager::SurfaceManager::publish_surfaces_mesh_pairs() const {
    Surfaces::Ptr surfaces_msg;
    if (this->surfaces_pub_.getNumSubscribers() > 0) {
        surfaces_msg = boost::make_shared<Surfaces>();
        surfaces_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        surfaces_msg->header.frame_id = this->target_frame_;
        surfaces_msg->surfaces.reserve(this->surfaces_.size());
    }

    SurfaceMeshes::Ptr surface_meshes_msg;
    if (this->surface_meshes_pub_.getNumSubscribers() > 0) {
        surface_meshes_msg = boost::make_shared<SurfaceMeshes>();
        surface_meshes_msg->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        surface_meshes_msg->header.frame_id = this->target_frame_;
        surface_meshes_msg->surface_meshes.reserve(this->surfaces_.size());
    }

    if (surfaces_msg && surface_meshes_msg) {
        auto surfaces_bi = back_inserter(surfaces_msg->surfaces);
        auto surface_meshes_bi = back_inserter(surface_meshes_msg->surface_meshes);
        for (const SurfaceMeshPair &pair : this->surfaces_) std::tie(*surfaces_bi, *surface_meshes_bi) = pair;
    } else if (surfaces_msg) {
        auto surfaces_bi = back_inserter(surfaces_msg->surfaces);
        for (const SurfaceMeshPair &pair : this->surfaces_) std::tie(*surfaces_bi, std::ignore) = pair;
    } else if (surface_meshes_msg) {
        auto surface_meshes_bi = back_inserter(surface_meshes_msg->surface_meshes);
        for (const SurfaceMeshPair &pair : this->surfaces_) std::tie(std::ignore, *surface_meshes_bi) = pair;
    }

    if (surfaces_msg) this->surfaces_pub_.publish(surfaces_msg);
    if (surface_meshes_msg) this->surface_meshes_pub_.publish(surface_meshes_msg);
}

void surface_manager::SurfaceManager::config_callback(SurfaceManagerConfig &config,
                                                        uint32_t level __attribute((unused))) {
    if (publish_interval_ != config.publish_interval) {
        publish_interval_ = static_cast<float>(config.publish_interval);
        publish_timer_.setPeriod(ros::Duration(publish_interval_));
        NODELET_DEBUG ("[config_callback] Setting publish interval: %f.", publish_interval_);
    }

    if (target_frame_ != config.target_frame) {
        target_frame_ = config.target_frame;
        NODELET_DEBUG ("[config_callback] Setting the target frame to: %s.", target_frame_.c_str());
    }
}

PLUGINLIB_EXPORT_CLASS(surface_manager::SurfaceManager, nodelet::Nodelet)
