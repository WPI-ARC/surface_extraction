//
// Created by will on 2/15/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_manager/SurfaceManager.h>

extern "C" {
#include <triangle.h>
}

#define INLIERS_TOPIC "new_surface_inliers"
#define CONCAVE_HULL_TOPIC "new_surface_concave_hull"
#define PLANE_TOPIC "new_surface_plane"
#define REPLACE_SURFACE_TOPIC "replace_surface"

#define tri_x(points, i) (points[i * 2])
#define tri_y(points, i) (points[i * 2 + 1])
#define tri_v1(triangles, i) (static_cast<uint32_t>(triangles[i * 3]))
#define tri_v2(triangles, i) (static_cast<uint32_t>(triangles[i * 3 + 1]))
#define tri_v3(triangles, i) (static_cast<uint32_t>(triangles[i * 3 + 2]))

Eigen::Affine3f tf_from_plane_model(const pcl_msgs::ModelCoefficients &plane) {
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

    // Start publisher timer
    // TODO: Configurable interval
    publish_timer_ = private_nh.createTimer(ros::Duration(1), bind(&SurfaceManager::publish, this, _1));


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
    std::stringstream s;
    s << "New surface has " << concave_hull->polygons.size() << " surfaces of sizes {";
    for (std::size_t i = 0; i < concave_hull->polygons.size() && i < 5; ++i) {
        if (i != 0) s << ", ";
        s << concave_hull->polygons[i].vertices.size();
    }
    if (concave_hull->polygons.size() > 5) s << ", ...";
    s << "}";
    NODELET_INFO("%s", s.str().c_str());

//    ros::Duration(20).sleep();

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
    surfaces[id].inliers = *inliers;

    surface_meshes[id].id = static_cast<unsigned int>(id);

//    this->make_3d_mesh(*concave_hull, tf_from_plane_model(*model));

//    pcl::EarClipping triangulate;
//    pcl::PolygonMesh::Ptr in_mesh = boost::make_shared<pcl::PolygonMesh>();
//    pcl::PolygonMesh out_mesh;
//    pcl_conversions::toPCL(*concave_hull, *in_mesh);
//    triangulate.setInputMesh(in_mesh);
//    triangulate.process(out_mesh);
//    pcl_conversions::fromPCL(out_mesh, surface_meshes[id].mesh);


//    NODELET_DEBUG_STREAM("[" << getName() << ":add_surface_synchronized] Triangle mesh finished with" << left_out <<
//                         "vertices unable to be converted");


//    std::sort(surfaces.begin(), surfaces.end(), [](const Surface& a, const Surface& b) {
//        auto a_size = a.concave_hull.cloud.width * a.concave_hull.cloud.height;
//        auto b_size = b.concave_hull.cloud.width * b.concave_hull.cloud.height;
//        // A should come first (compare lower) if its size is greater than B's
//        return a_size > b_size;
//    });
}

void surface_manager::SurfaceManager::publish(const ros::TimerEvent &event) {
    if (surfaces.size() < 1) return;

    std_msgs::Header pub_header;
    pub_header.stamp = ros::Time::now();
    pub_header.frame_id = "/base_link"; // TODO: Don't hardcode this


    if (surfaces_pub_.getNumSubscribers() > 0) {
        Surfaces::Ptr surfaces_msg = boost::make_shared<Surfaces>();
        surfaces_msg->header = pub_header;
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
        surface_meshes_msg->header = pub_header;
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

        pcl_conversions::toPCL(pub_header, output->header);
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

        pcl_conversions::toPCL(pub_header, output->header);
        perimeter_pub_.publish(output);
    }

    if (visualization_pub_.getNumSubscribers() > 0) {
        vis::MarkerArray all_markers;

        for (Surface surface : surfaces) {
            vis::Marker perimeter;
            perimeter.header = pub_header;
            perimeter.ns = "surface_perimeter";
            perimeter.id = surface.id;
            perimeter.type = vis::Marker::LINE_LIST;
            perimeter.action = vis::Marker::MODIFY;
            // perimeter.pose not needed
            perimeter.scale.x = 0.01; // scale.x controls the width of the line segments
            perimeter.color = surface.color;

            sensor_msgs::PointCloud2Iterator<float> iter_x(surface.concave_hull.cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(surface.concave_hull.cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(surface.concave_hull.cloud, "z");

            for (pcl_msgs::Vertices &polygon : surface.concave_hull.polygons) {
                for (std::size_t i = 0; i < polygon.vertices.size(); i++) {
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

        vis::MarkerArray markers;
        for (Surface surface : surfaces) {
            shape_msgs::Mesh trimesh = this->make_3d_mesh(surface.concave_hull, tf_from_plane_model(surface.model));
            vis::Marker marker;
            marker.header = surface.concave_hull.header;
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
            for (auto triangle : trimesh.triangles) {
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[0]]);
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[1]]);
                marker.points.push_back(trimesh.vertices[triangle.vertex_indices[2]]);
            }

            markers.markers.push_back(marker);
        }

        visualization_pub_.publish(markers);
    }

    NODELET_DEBUG("SurfaceManager published surfaces and visualizations");
}

void surface_manager::SurfaceManager::replace_surface(const SurfaceStamped::ConstPtr new_surface) {
    NODELET_DEBUG("Replacement for surface %u received", new_surface->surface.id);
    auto old_surface = std::find_if(surfaces.begin(), surfaces.end(), [&new_surface](Surface &test_surface) {
        return test_surface.id == new_surface->surface.id;
    });

    NODELET_DEBUG_STREAM("Old version of surface found at " << std::distance(surfaces.begin(), old_surface) <<
                         " (length of array is " << surfaces.size() << ")");

    *old_surface = new_surface->surface;
}

shape_msgs::Mesh surface_manager::SurfaceManager::make_3d_mesh(PolygonMesh hull, Eigen::Affine3f tf) {
    shape_msgs::Mesh trimesh;
    auto cloud_size = hull.cloud.width * hull.cloud.height;


    NODELET_DEBUG("Triangulating surface using Triangle");

    // Populate points_2d and trimesh.vertices
    double points_2d[cloud_size * 2];
    trimesh.vertices.resize(cloud_size * 2); // trimesh will contain 2 copies of each point
    sensor_msgs::PointCloud2Iterator<float> iter_x(hull.cloud, "x"), iter_y(hull.cloud, "y"), iter_z(hull.cloud, "z");
    for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
        Eigen::Vector3f pt = Eigen::Vector3f(*iter_x, *iter_y, *iter_z);
        Eigen::Vector3f flat_pt = tf * pt;
        tri_x(points_2d, i) = flat_pt[0];
        tri_y(points_2d, i) = flat_pt[1];

        // Add vertices for the top surface
        trimesh.vertices[i].x = pt[0];
        trimesh.vertices[i].y = pt[1];
        trimesh.vertices[i].z = pt[2];

        // TODO: Don't build a new offset vector each time, don't hardcode offset size
        Eigen::Vector3f offset_pt = pt + tf.linear() * Eigen::Vector3f(0, 0, -0.02);

        // Add vertices for the bottom surface
        trimesh.vertices[i + cloud_size].x = offset_pt[0];
        trimesh.vertices[i + cloud_size].y = offset_pt[1];
        trimesh.vertices[i + cloud_size].z = offset_pt[2];
    }

    // Populate segment list and holes
    std::vector<int> segment_list;
    double holes[(hull.polygons.size() - 1) * 2];
    int hole_num = -1;
    for (pcl_msgs::Vertices &polygon : hull.polygons) {
        auto polysize = polygon.vertices.size();

        // Whether it's a hole or not, the segments will be added, so make room for them
        segment_list.reserve(segment_list.size() + polysize);
        if (hole_num == -1) { // Assumption: the first polygon is the outer perimeter and all subsequent ones are holes
            // Add the segments
            for (int i = 0; i < polysize; i++) {
                segment_list.push_back(polygon.vertices[i]);
                segment_list.push_back(polygon.vertices[(i + 1) % polysize]);
            }
        } else {
            // min and max x must be initialized to the index of any point within the current polygon
            int min_x = polygon.vertices[0], max_x = polygon.vertices[0];
            // Add the segments and find the min and max x-coordinate
            for (int i = 0; i < polysize; i++) {
                segment_list.push_back(polygon.vertices[i]);
                segment_list.push_back(polygon.vertices[(i + 1) % polysize]);

                if (tri_x(points_2d, polygon.vertices[i]) < tri_x(points_2d, min_x)) min_x = polygon.vertices[i];
                if (tri_x(points_2d, polygon.vertices[i]) > tri_x(points_2d, max_x)) max_x = polygon.vertices[i];
            }

            double mid_x = (tri_x(points_2d, min_x) + tri_x(points_2d, max_x)) / 2.;
            NODELET_DEBUG_STREAM(
                    "min_x = " << tri_x(points_2d, min_x) << ", max_x = " << tri_x(points_2d, max_x) << ", mid_x = " <<
                    mid_x);
            std::vector<double> intersection_ys;
            // Find all lines which intersect with x = mid_x and save the y-coordinate of the intersection
            for (int i = 0; i < polysize; i++) {
                auto point_a = polygon.vertices[i], point_b = polygon.vertices[(i + 1) % polysize];
                auto x_a = tri_x(points_2d, point_a), x_b = tri_x(points_2d, point_b);
                if ((x_a < mid_x && x_b > mid_x) || (x_a > mid_x && x_b < mid_x)) {
                    auto y_a = tri_y(points_2d, point_a), y_b = tri_y(points_2d, point_b);
                    intersection_ys.push_back((y_b - y_a) / (x_b - x_a) * (mid_x - x_a) + y_a);
                    NODELET_DEBUG_STREAM(
                            "Found intersection with " << point_a << " (" << x_a << ", " << y_a << "), " << point_b <<
                            " (" << x_b << ", " << y_b << ") at (" << mid_x << ", " << intersection_ys.back() << ")");
                } else {
                    auto y_a = tri_y(points_2d, point_a), y_b = tri_y(points_2d, point_b);
                    NODELET_DEBUG_STREAM(
                            "   No intersection with " << point_a << " (" << x_a << ", " << y_a << "), " << point_b <<
                            " (" << x_b << ", " << y_b << ") and x = " << mid_x);
                }
            }
            assert(intersection_ys.size() >= 2);
            std::partial_sort(intersection_ys.begin(), intersection_ys.begin() + 2, intersection_ys.end());

            // The midpoint of the first two intersections is guaranteed to be inside the polygon
            tri_x(holes, hole_num) = mid_x;
            tri_y(holes, hole_num) = (intersection_ys[0] + intersection_ys[1]) / 2.;
        }
        hole_num++;
    }

    struct triangulateio tri_in, tri_out;

    // Initialize tri_in according to triangle.h:175
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-stack-address"
    tri_in.pointlist = points_2d;
    tri_in.numberofpoints = cloud_size;
    tri_in.pointmarkerlist = NULL;
    tri_in.numberofpointattributes = 0;
    tri_in.pointattributelist = NULL;
    tri_in.segmentlist = &segment_list[0];
    tri_in.numberofsegments = static_cast<int>(segment_list.size()) / 2;
    tri_in.segmentmarkerlist = NULL;
    tri_in.numberofholes = static_cast<int>(hull.polygons.size() - 1);
    tri_in.holelist = holes;
//    tri_in.numberofholes = 0;
//    tri_in.holelist = NULL;
    tri_in.numberofregions = 0;
    tri_in.regionlist = NULL;
#pragma clang diagnostic pop

    // Initialize tri_out according to triangle.h:206
    tri_out.trianglelist = NULL; // NULL tells Triangle to allocate the memory for it

    // Triangulate from the triangle library, switches are:
    // z: makes indices start counting from zero, same convention as PCL
    // p: tells it to triangulate a polygon
    // N: don't output nodes to save memory (if this changes, initialize tri_out.pointlist)
    // P: don't output the segments to save memory (if this changes, initialize tri_out.segmentlist)
    // B: don't output the boundary markers to save memory (if this changes, initialize tri_out.segmentmarkerlist)
    // Q: (not used while debugging) don't print output
    // IMPORTANT: If the flags change, make sure to change tri_in and tri_out accordingly
    triangulate("zpNPB", &tri_in, &tri_out, NULL);
    assert(tri_out.numberofcorners == 3);

    // Output gets 2 triangles per tri_out triangle (one for the top layer and one for the bottom), plus 2 triangles
    // per edge segment (to join the layers). Note that segment_list is already (2 * # of edge segments).
    trimesh.triangles.resize(static_cast<std::size_t>(tri_out.numberoftriangles) * 2 + segment_list.size());

    // I suspect this may be doable with a memcpy but I'm not sure how to check
    for (int i = 0; i < tri_out.numberoftriangles; i += 1) {
        // Add triangles for the top surface
        trimesh.triangles[i].vertex_indices = {tri_v1(tri_out.trianglelist, i),
                                               tri_v2(tri_out.trianglelist, i),
                                               tri_v3(tri_out.trianglelist, i)};

        // Add triangles for the bottom surface
        auto offset_i = i + tri_out.numberoftriangles;
        trimesh.triangles[offset_i].vertex_indices = {tri_v1(tri_out.trianglelist, i) + cloud_size,
                                                      tri_v2(tri_out.trianglelist, i) + cloud_size,
                                                      tri_v3(tri_out.trianglelist, i) + cloud_size};

    }

    int offset = tri_out.numberoftriangles * 2;
    for (pcl_msgs::Vertices &polygon : hull.polygons) {
        auto polysize = polygon.vertices.size();
        for (int i = 0; i < polysize; i++) {
            auto next_i = (i + 1) % polysize;
            trimesh.triangles[offset + i*2].vertex_indices = {polygon.vertices[next_i] + cloud_size,
                                                              polygon.vertices[i] + cloud_size,
                                                              polygon.vertices[i]};
            trimesh.triangles[offset + i*2 + 1].vertex_indices = {polygon.vertices[i],
                                                                  polygon.vertices[next_i],
                                                                  polygon.vertices[next_i] + cloud_size};
        }
        offset += polysize * 2;
    }

    trifree(tri_out.trianglelist);

    return trimesh;
}


PLUGINLIB_EXPORT_CLASS(surface_manager::SurfaceManager, nodelet::Nodelet)
