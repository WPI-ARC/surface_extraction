//
// Created by will on 2/15/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_manager/SurfaceManager.h>


#define INLIERS_TOPIC "new_surface_inliers"
#define CONCAVE_HULL_TOPIC "new_surface_concave_hull"
#define PLANE_TOPIC "new_surface_plane"
#define REPLACE_SURFACE_TOPIC "replace_surface"

void surface_manager::SurfaceManager::onInit() {
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    int max_queue_size = 3;
    if (!private_nh.getParam("max_queue_size", max_queue_size)) {
        max_queue_size = 3;
    }

    // Advertise a topic called "surfaces" with a queue size of max_queue_size and latched=true
    surfaces_pub_ = private_nh.advertise<Surfaces>("surfaces", (uint32_t) max_queue_size, true);
//    surface_clouds_pub_ = private_nh.advertise<SurfaceClouds>("surface_clouds", (uint32_t) max_queue_size, true);
    surface_meshes_pub_ = private_nh.advertise<SurfaceMeshes>("surface_meshes", (uint32_t) max_queue_size, true);
    output_pub_ = private_nh.advertise<PointCloudOut>("output", (uint32_t) max_queue_size);
    perimeter_pub_ = private_nh.advertise<PointCloudOut>("perimeter", (uint32_t) max_queue_size);
    visualization_pub_ = private_nh.advertise<vis::MarkerArray>("surfaces_visualization", (uint32_t) max_queue_size);

    // Subscribe to new_surface_* inputs (for adding surfaces)
    new_surface_inliers_sub_.subscribe(private_nh, INLIERS_TOPIC, (uint32_t) max_queue_size);
    new_surface_convex_hull_sub_.subscribe(private_nh, CONCAVE_HULL_TOPIC, (uint32_t) max_queue_size);
    new_surface_plane_sub_.subscribe(private_nh, PLANE_TOPIC, (uint32_t) max_queue_size);

    replace_surface_sub_ = private_nh.subscribe<SurfaceStamped>(REPLACE_SURFACE_TOPIC, (uint32_t) max_queue_size,
                                                                bind(&SurfaceManager::replace_surface, this, _1));

    new_surface_synchronizer_ = boost::make_shared<NewSurfaceSynchronizer>(max_queue_size);
    new_surface_synchronizer_->connectInput(new_surface_inliers_sub_, new_surface_convex_hull_sub_,
                                            new_surface_plane_sub_);
    new_surface_synchronizer_->registerCallback(bind(&SurfaceManager::add_surface_synchronized, this, _1, _2, _3));

    NODELET_DEBUG("[%s::onInit] SurfaceManager started with max_queue_size = %d\n", getName().c_str(), max_queue_size);

    NODELET_DEBUG("[%s::onInit] Subscribed to topics %s, %s, %s", getName().c_str(),
                  private_nh.resolveName(INLIERS_TOPIC).c_str(),
                  private_nh.resolveName(CONCAVE_HULL_TOPIC).c_str(),
                  private_nh.resolveName(PLANE_TOPIC).c_str());
}

void surface_manager::SurfaceManager::add_surface_synchronized(const PointCloudIn::ConstPtr inliers,
                                                               const PolygonMesh::ConstPtr concave_hull,
                                                               const ModelCoefficients::ConstPtr model) {
    auto cloud_size = inliers->width * inliers->height;
    auto hull_size = concave_hull->cloud.width * concave_hull->cloud.height;
    NODELET_DEBUG("New Surface received with %u points, %u vertices", cloud_size, hull_size);

    unsigned long id = surfaces.size();
    surfaces.resize(id + 1);
//    surface_clouds.resize(id+1);
    surface_meshes.resize(id + 1);

    surfaces[id].id = static_cast<unsigned int>(id);
    surfaces[id].color = std_msgs::ColorRGBA();
    surfaces[id].color.r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    surfaces[id].color.a = 1;
    surfaces[id].model = *model;
    surfaces[id].concave_hull = *concave_hull;

//    surface_clouds[id].id = static_cast<unsigned int>(id);
    surfaces[id].inliers = *inliers;


    surface_meshes[id].id = static_cast<unsigned int>(id);
    // Get triangle mesh

//    NODELET_DEBUG_STREAM("[" << getName() << ":add_surface_synchronized] Triangle mesh finished with" << left_out <<
//                         "vertices unable to be converted");


//    std::sort(surfaces.begin(), surfaces.end(), [](const Surface& a, const Surface& b) {
//        auto a_size = a.concave_hull.cloud.width * a.concave_hull.cloud.height;
//        auto b_size = b.concave_hull.cloud.width * b.concave_hull.cloud.height;
//        // A should come first (compare lower) if its size is greater than B's
//        return a_size > b_size;
//    });

    this->publish(concave_hull->header);
}

void surface_manager::SurfaceManager::publish(std_msgs::Header header) {
    if (surfaces_pub_.getNumSubscribers() > 0) {
        Surfaces::Ptr surfaces_msg = boost::make_shared<Surfaces>();
        surfaces_msg->header = header;
        surfaces_msg->surfaces = surfaces;
        surfaces_pub_.publish(surfaces_msg);
    }

//    if (surface_clouds_pub_.getNumSubscribers() > 0) {
//        SurfaceClouds::Ptr surface_clouds_msg = boost::make_shared<SurfaceClouds>();
//        surface_clouds_msg->header = header;
//        surface_clouds_msg->surfaces = surface_clouds;
//        surface_clouds_pub_.publish(surface_clouds_msg);
//    }

    if (surface_meshes_pub_.getNumSubscribers() > 0) {
        SurfaceMeshes::Ptr surface_meshes_msg = boost::make_shared<SurfaceMeshes>();
        surface_meshes_msg->header = header;
        surface_meshes_msg->surfaces = surface_meshes;
        surface_meshes_pub_.publish(surface_meshes_msg);
    }

    if (output_pub_.getNumSubscribers() > 0) {
        PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();

        for (Surface surface : surfaces) {
            size_t outputPointsSize = output->points.size();
            size_t projectedPointsSize = surface.inliers.height * surface.inliers.width;
            output->points.resize(outputPointsSize + projectedPointsSize);

            // C++ makes me sad
            sensor_msgs::PointCloud2Iterator<float> iter_x(surface.inliers, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(surface.inliers, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(surface.inliers, "z");

            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
                output->points[outputPointsSize + i].x = *iter_x;
                output->points[outputPointsSize + i].y = *iter_y;
                output->points[outputPointsSize + i].z = *iter_z;
                output->points[outputPointsSize + i].r = static_cast<uint8_t>(surface.color.r * 255);
                output->points[outputPointsSize + i].g = static_cast<uint8_t>(surface.color.g * 255);
                output->points[outputPointsSize + i].b = static_cast<uint8_t>(surface.color.b * 255);
            }
        }

        pcl_conversions::toPCL(header, output->header);
        output_pub_.publish(output);
    }

    if (perimeter_pub_.getNumSubscribers() > 0) {
        PointCloudOut::Ptr output = boost::make_shared<PointCloudOut>();

        for (Surface surface : surfaces) {
            size_t outputPointsSize = output->points.size();
            size_t projectedPointsSize = surface.concave_hull.cloud.height * surface.concave_hull.cloud.width;
            output->points.resize(outputPointsSize + projectedPointsSize);

            // C++ makes me sad
            sensor_msgs::PointCloud2Iterator<float> iter_x(surface.concave_hull.cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(surface.concave_hull.cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(surface.concave_hull.cloud, "z");

            for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
                output->points[outputPointsSize + i].x = *iter_x;
                output->points[outputPointsSize + i].y = *iter_y;
                output->points[outputPointsSize + i].z = *iter_z;
                output->points[outputPointsSize + i].r = static_cast<uint8_t>(surface.color.r * 255);
                output->points[outputPointsSize + i].g = static_cast<uint8_t>(surface.color.g * 255);
                output->points[outputPointsSize + i].b = static_cast<uint8_t>(surface.color.b * 255);
            }
        }

        pcl_conversions::toPCL(header, output->header);
        perimeter_pub_.publish(output);
    }

    if (visualization_pub_.getNumSubscribers() > 0) {
        vis::MarkerArray all_markers;

        for (Surface surface : surfaces) {
            vis::Marker perimeter;
            perimeter.header = header;
            perimeter.ns = "surface_perimeter";
            perimeter.id = surface.id;
            perimeter.type = vis::Marker::LINE_LIST;
            perimeter.action = vis::Marker::MODIFY;
            // perimeter.pose not needed
            perimeter.scale.x = 0.05; // scale.x controls the width of the line segments
            perimeter.color = surface.color;

            sensor_msgs::PointCloud2Iterator<float> iter_x(surface.concave_hull.cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(surface.concave_hull.cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(surface.concave_hull.cloud, "z");

            for (pcl_msgs::Vertices& polygon : surface.concave_hull.polygons) {
                for (int i = 0; i < polygon.vertices.size(); i++) {
                    geometry_msgs::Point pointA, pointB;
                    pointA.x = *(iter_x + polygon.vertices[i]);
                    pointA.y = *(iter_y + polygon.vertices[i]);
                    pointA.z = *(iter_z + polygon.vertices[i]);

                    // Connect each point to the next, and the last point back to the first
                    unsigned long next_i = (i + 1) % polygon.vertices.size();

                    pointB.x = *(iter_x + polygon.vertices[next_i]);
                    pointB.y = *(iter_y + polygon.vertices[next_i]);
                    pointB.z = *(iter_z + polygon.vertices[next_i]);

                    perimeter.points.push_back(pointA);
                    perimeter.points.push_back(pointB);
                }
            }

            all_markers.markers.push_back(perimeter);
        }

        visualization_pub_.publish(all_markers);
    }

    NODELET_DEBUG("SurfaceManager published surfaces on all topics with any subscribers");
}

void surface_manager::SurfaceManager::replace_surface(const SurfaceStamped::ConstPtr new_surface) {
    NODELET_DEBUG("Replacement for surface %u received", new_surface->surface.id);
    auto old_surface = std::find_if(surfaces.begin(), surfaces.end(), [&new_surface](Surface &test_surface) {
        return test_surface.id == new_surface->surface.id;
    });

    NODELET_DEBUG_STREAM("Old version of surface found at " << std::distance(surfaces.begin(), old_surface) <<
                         " (length of array is " << surfaces.size() << ")");

    *old_surface = new_surface->surface;

    this->publish(new_surface->header);
}


PLUGINLIB_EXPORT_CLASS(surface_manager::SurfaceManager, nodelet::Nodelet)
