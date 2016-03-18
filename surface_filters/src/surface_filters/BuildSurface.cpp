/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/BuildSurface.h>

extern "C" {
#include <triangle.h>
}

void surface_filters::BuildSurface::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Advertise output nodes
    pub_new_surface = pnh_->advertise<Surface>("new_surface", max_queue_size_);
    pub_updated_surface = pnh_->advertise<Surface>("updated_surface", max_queue_size_);
    pub_new_surface_mesh = pnh_->advertise<SurfaceMesh>("new_surface_mesh", max_queue_size_);
    pub_updated_surface_mesh = pnh_->advertise<SurfaceMesh>("updated_surface_mesh", max_queue_size_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<BuildSurfaceConfig> >(*pnh_);
    srv_->setCallback(bind(&BuildSurface::config_callback, this, _1, _2));

    int model_type_int;
    if (!pnh_->getParam("model_type", model_type_int)) {
        NODELET_ERROR ("[%s::onInit] Need a 'model_type' parameter to be set before continuing!", getName().c_str());
        return;
    }
    model_type_ = surfaces::sacModelFromConfigInt(model_type_int);
    proj_.setModelType(model_type_);

    sub_create_surface_ = pnh_->subscribe<Segment>("create_surface", max_queue_size_,
                                                   bind(&BuildSurface::synchronized_input_callback, this, _1, true));
    sub_update_surface_ = pnh_->subscribe<Segment>("update_surface", max_queue_size_,
                                                   bind(&BuildSurface::synchronized_input_callback, this, _1, false));


    NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                           " - use_indices    : %s\n",
                   getName().c_str(),
                   (use_indices_) ? "true" : "false");
}

//////////////////////////////////////////////////////////////////////////////////////////////
void surface_filters::BuildSurface::synchronized_input_callback(const Segment::ConstPtr &segment, bool is_new) {

    // TODO: Check that inputs are valid

    NODELET_DEBUG(
            "[%s::synchronized_input_callback] PointCloud with %lu data points, stamp %f, and frame %s on topic %s received.",
            getName().c_str(), segment->inliers.size(), fromPCL(segment->header).stamp.toSec(),
            segment->header.frame_id.c_str(),
            getMTPrivateNodeHandle().resolveName(is_new ? "create_surface" : "update_surface").c_str());


    Surface::Ptr surface = this->publish_surface(segment, is_new);
    this->publish_surface_mesh(surface, is_new);
}

surface_filters::BuildSurface::Surface::Ptr surface_filters::BuildSurface::publish_surface(
        const Segment::ConstPtr &segment, bool is_new) {
    ros::WallTime start = ros::WallTime::now();

    // Create output objects
    Surface::Ptr surface = boost::make_shared<Surface>();
    surface->header = segment->header;
    if (!is_new) surface->surface.id = segment->surface_id;
    surface->surface.model = segment->model;
    this->get_projected_cloud(segment, surface->surface.inliers);
    this->get_concave_hull({surface, &surface->surface.inliers}, surface->surface.concave_hull);

    (is_new ? this->pub_new_surface : this->pub_updated_surface).publish(surface);

    auto num_pts = surface->surface.inliers.size();
    auto num_vertices = surface->surface.concave_hull.cloud.height * surface->surface.concave_hull.cloud.width;
    NODELET_INFO_STREAM("[" << getName().c_str() << "::synchronized_input_callback] " << std::setprecision(3) <<
                        "(" << (ros::WallTime::now() - start) << " sec" <<
                        ", " << num_pts << " points" <<
                        ", " << num_vertices << " vertices" <<
                        ") Published " << (is_new ? "new" : "updated") << " surface");

    return surface;
}

surface_filters::BuildSurface::SurfaceMesh::Ptr surface_filters::BuildSurface::publish_surface_mesh(
        const Surface::ConstPtr &surface, bool is_new) {
    ros::WallTime start = ros::WallTime::now();

    // Create output objects
    SurfaceMesh::Ptr surface_mesh = boost::make_shared<SurfaceMesh>();
    surface_mesh->header = surface->header;
    this->get_3d_mesh(surface, surface_mesh->surface_mesh.surface_mesh);

    (is_new ? this->pub_new_surface_mesh : this->pub_updated_surface_mesh).publish(surface_mesh);

    auto num_triangles = surface_mesh->surface_mesh.surface_mesh.triangles.size();
    NODELET_INFO_STREAM("[" << getName().c_str() << "::synchronized_input_callback] " << std::setprecision(3) <<
                        "(" << (ros::WallTime::now() - start) << " sec, " << num_triangles << " triangles) " <<
                        ") Published " << (is_new ? "new" : "updated") << " surface mesh");

    return surface_mesh;
}

void surface_filters::BuildSurface::config_callback(BuildSurfaceConfig &config, uint32_t level __attribute__((unused))) {
    if (model_type_ != config.model_type) {
        model_type_ = surfaces::sacModelFromConfigInt(config.model_type);
        proj_.setModelType(model_type_);
        NODELET_DEBUG ("[config_callback] Setting the model type to: %d.", model_type_);
    }

    if (dimension_ != config.dimension) {
        dimension_ = config.dimension;
        NODELET_DEBUG ("[config_callback] Setting the dimension to: %d.", dimension_);
        hull_.setDimension(dimension_);
    }
    if (alpha_ != config.alpha) {
        alpha_ = config.alpha;
        NODELET_DEBUG ("[config_callback] Setting the alpha parameter to: %f.", alpha_);
        hull_.setAlpha(alpha_);
    }
}

void surface_filters::BuildSurface::get_concave_hull(const PointCloudIn::ConstPtr &projected_cloud,
                                                     PolygonMesh &chull_output) {
    // Annoyingly, concave hull is not thread-safe.
    std::unique_lock<std::mutex> hull_lock(hull_mutex_);

    // Pass inputs to PCL
    hull_.setInputCloud(projected_cloud);

    // Do the reconstruction
    hull_.reconstruct(chull_output);
}

void surface_filters::BuildSurface::get_projected_cloud(const Segment::ConstPtr &segment,
                                                        PointCloudIn &proj_output) {
    // Uses shared_ptrs' alaiasing constructor to get a shared pointer to the inliers cloud and to the model coeffs
    // that cooperate with the shared pointer to the segment
    proj_.setInputCloud(PointCloudIn::ConstPtr(segment, &segment->inliers));
    proj_.setModelCoefficients(pcl::ModelCoefficients::ConstPtr(segment, &segment->model));
    proj_.filter(proj_output);
}

void surface_filters::BuildSurface::get_3d_mesh(const Surface::ConstPtr &surface, shape_msgs::Mesh &output_trimesh) {
    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(surface->surface.concave_hull.cloud, hull_cloud);
    assert(hull_cloud.size() == surface->surface.concave_hull.cloud.width * surface->surface.concave_hull.cloud.height);

    std::vector<double> points_2d;
    std::vector<int> segment_list;
    std::vector<double> holes;

    this->get_triangulation_vertices(surface->surface.model, hull_cloud, output_trimesh, points_2d);
    assert(points_2d.size() == hull_cloud.size()  * 2);
    this->get_triangulation_segments(surface->surface.concave_hull, points_2d, segment_list, holes);
    assert(points_2d.size() == hull_cloud.size()  * 2);
    this->get_triangluation_trimesh(points_2d, segment_list, holes, surface->surface.concave_hull, output_trimesh);
}

void surface_filters::BuildSurface::get_triangulation_vertices(const pcl::ModelCoefficients &model,
                                                               const PointCloudIn &hull_cloud,
                                                               shape_msgs::Mesh &output_trimesh,
                                                               std::vector<double> &points_2d) const {
    auto cloud_size = hull_cloud.size();
    Eigen::Affine3f tf = surfaces::tf_from_plane_model(model);
    Eigen::Vector3f offset_vect = tf.linear() * Eigen::Vector3f(0, 0, -0.02);

    // Populate points_2d and trimesh.vertices
    output_trimesh.vertices.resize(cloud_size * 2); // trimesh will contain 2 copies of each point
    points_2d.resize(cloud_size * 2);
    for (size_t i = 0; i < hull_cloud.size(); i++) {
        Eigen::Vector3f pt = Eigen::Vector3f(hull_cloud[i].x, hull_cloud[i].y, hull_cloud[i].z);
        Eigen::Vector3f flat_pt = tf * pt;
        tri_x(points_2d, i) = flat_pt[0];
        tri_y(points_2d, i) = flat_pt[1];

        // Add vertices for the top surface
        output_trimesh.vertices[i].x = pt[0];
        output_trimesh.vertices[i].y = pt[1];
        output_trimesh.vertices[i].z = pt[2];

        Eigen::Vector3f offset_pt = pt + offset_vect;

        // Add vertices for the bottom surface
        output_trimesh.vertices[i + cloud_size].x = offset_pt[0];
        output_trimesh.vertices[i + cloud_size].y = offset_pt[1];
        output_trimesh.vertices[i + cloud_size].z = offset_pt[2];
    }
}

void surface_filters::BuildSurface::get_triangulation_segments(const PolygonMesh &hull,
                                                               const std::vector<double> &points_2d,
                                                               std::vector<int> &segment_list,
                                                               std::vector<double> &holes) const {
    holes.resize((hull.polygons.size() - 1) * 2);
    // Populate segment list and holes
    int hole_num = -1;
    for (const pcl::Vertices &polygon : hull.polygons) {
        auto polysize = polygon.vertices.size();

        // Whether it's a hole or not, the segments will be added, so make room for them
        segment_list.reserve(segment_list.size() + polysize);
        if (hole_num == -1) { // Assumption: the first polygon is the outer perimeter and all subsequent ones are holes
            // Add the segments
            for (int i = 0; i < static_cast<int>(polysize); i++) {
                segment_list.push_back(polygon.vertices[i]);
                segment_list.push_back(polygon.vertices[(i + 1) % polysize]);
            }
        } else {
            // min and max x must be initialized to the index of any point within the current polygon
            int min_x = polygon.vertices[0], max_x = polygon.vertices[0];
            // Add the segments and find the min and max x-coordinate
            for (int i = 0; i < static_cast<int>(polysize); i++) {
                segment_list.push_back(polygon.vertices[i]);
                segment_list.push_back(polygon.vertices[(i + 1) % polysize]);

                if (tri_x(points_2d, polygon.vertices[i]) < tri_x(points_2d, min_x)) min_x = polygon.vertices[i];
                if (tri_x(points_2d, polygon.vertices[i]) > tri_x(points_2d, max_x)) max_x = polygon.vertices[i];
            }

            double mid_x = (tri_x(points_2d, min_x) + tri_x(points_2d, max_x)) / 2.;
            std::vector<double> intersection_ys;
            // Find all lines which intersect with x = mid_x and save the y-coordinate of the intersection
            for (int i = 0; i < static_cast<int>(polysize); i++) {
                auto point_a = polygon.vertices[i], point_b = polygon.vertices[(i + 1) % polysize];
                auto x_a = tri_x(points_2d, point_a), x_b = tri_x(points_2d, point_b);
                if ((x_a < mid_x && x_b > mid_x) || (x_a > mid_x && x_b < mid_x)) {
                    auto y_a = tri_y(points_2d, point_a), y_b = tri_y(points_2d, point_b);
                    intersection_ys.push_back((y_b - y_a) / (x_b - x_a) * (mid_x - x_a) + y_a);
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
}

void surface_filters::BuildSurface::get_triangluation_trimesh(std::vector<double> &points_2d,
                                                              std::vector<int> &segment_list,
                                                              std::vector<double> &holes, const PolygonMesh &hull,
                                                              shape_msgs::Mesh &output_trimesh) const {
    struct triangulateio tri_in, tri_out;

    // Initialize tri_in according to triangle.h:175
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-stack-address"
    tri_in.pointlist = &points_2d[0];
    tri_in.numberofpoints = static_cast<int>(points_2d.size() / 2);
    tri_in.pointmarkerlist = NULL;
    tri_in.numberofpointattributes = 0;
    tri_in.pointattributelist = NULL;
    tri_in.segmentlist = &segment_list[0];
    tri_in.numberofsegments = static_cast<int>(segment_list.size()) / 2;
    tri_in.segmentmarkerlist = NULL;
    tri_in.numberofholes = static_cast<int>(holes.size() / 2);
    tri_in.holelist = &holes[0];
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
    triangulate("zpNPBQ", &tri_in, &tri_out, NULL);
    assert(tri_out.numberofcorners == 3);

    // Output gets 2 triangles per tri_out triangle (one for the top layer and one for the bottom), plus 2 triangles
    // per edge segment (to join the layers). Note that segment_list is already (2 * # of edge segments).
    output_trimesh.triangles.resize(static_cast<std::size_t>(tri_out.numberoftriangles) * 2 + segment_list.size());

    int cloud_size = static_cast<int>(output_trimesh.vertices.size()) / 2;

    // I suspect this may be doable with a memcpy but I'm not sure how to check
    for (int i = 0; i < tri_out.numberoftriangles; i += 1) {
        // Add triangles for the top surface
        output_trimesh.triangles[i].vertex_indices = {{tri_v1(tri_out.trianglelist, i),
                                                       tri_v2(tri_out.trianglelist, i),
                                                       tri_v3(tri_out.trianglelist, i)}};

        // Add triangles for the bottom surface
        auto offset_i = i + tri_out.numberoftriangles;
        output_trimesh.triangles[offset_i].vertex_indices = {{tri_v1(tri_out.trianglelist, i) + cloud_size,
                                                              tri_v2(tri_out.trianglelist, i) + cloud_size,
                                                              tri_v3(tri_out.trianglelist, i) + cloud_size}};
    }

    int offset = tri_out.numberoftriangles * 2;
    for (const pcl::Vertices &polygon : hull.polygons) {
        auto polysize = polygon.vertices.size();
        for (int i = 0; i < static_cast<int>(polysize); i++) {
            auto next_i = (i + 1) % polysize;
            output_trimesh.triangles[offset + i * 2].vertex_indices = {{polygon.vertices[next_i] + cloud_size,
                                                                        polygon.vertices[i] + cloud_size,
                                                                        polygon.vertices[i]}};
            output_trimesh.triangles[offset + i * 2 + 1].vertex_indices = {{polygon.vertices[i],
                                                                            polygon.vertices[next_i],
                                                                            polygon.vertices[next_i] + cloud_size}};
        }
        offset += polysize * 2;
    }

    trifree(tri_out.trianglelist);
}


PLUGINLIB_EXPORT_CLASS(surface_filters::BuildSurface, nodelet::Nodelet)
