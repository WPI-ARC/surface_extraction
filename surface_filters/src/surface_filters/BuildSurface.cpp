/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 * Using framework from the pcl_ros package at http://wiki.ros.org/pcl_ros
 */

#include <pluginlib/class_list_macros.h>
#include <surface_filters/BuildSurface.h>
#include <pcl/common/time.h>

using libtriangle::Triangulate;
using libtriangle::AdjacencyList;

void surface_filters::BuildSurface::onInit() {
    pcl_ros::PCLNodelet::onInit();

    // Advertise output nodes
    pub_new_surface = pnh_->advertise<SurfaceStamped>("new_surface", max_queue_size_);
    pub_updated_surface = pnh_->advertise<SurfaceStamped>("updated_surface", max_queue_size_);
    pub_new_surface_mesh = pnh_->advertise<SurfaceMeshStamped>("new_surface_mesh", max_queue_size_);
    pub_updated_surface_mesh = pnh_->advertise<SurfaceMeshStamped>("updated_surface_mesh", max_queue_size_);

    pub_markers = pnh_->advertise<visualization_msgs::MarkerArray>("makers", max_queue_size_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<BuildSurfaceConfig> >(*pnh_);
    srv_->setCallback(bind(&BuildSurface::config_callback, this, _1, _2));

    int model_type_int;
    if (!pnh_->getParam("model_type", model_type_int)) {
        NODELET_ERROR ("[%s::onInit] Need a 'model_type' parameter to be set before continuing!", getName().c_str());
        return;
    }

    pnh_->getParam("dimension", dimension_);
    pnh_->getParam("alpha", alpha_);

    model_type_ = surfaces::sacModelFromConfigInt(model_type_int);
    proj_.setModelType(model_type_);
    hull_.setDimension(dimension_);
    hull_.setAlpha(alpha_);

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

//    NODELET_DEBUG(
//            "[%s::synchronized_input_callback] PointCloud with %lu data points, stamp %f, and frame %s on topic %s received.",
//            getName().c_str(), segment->inliers.size(), fromPCL(segment->header).stamp.toSec(),
//            segment->header.frame_id.c_str(),
//            getMTPrivateNodeHandle().resolveName(is_new ? "create_surface" : "update_surface").c_str());


    SurfaceStamped::Ptr surface = this->publish_surface(segment, is_new);
    if (surface) this->publish_surface_mesh(surface, is_new);
}

auto surface_filters::BuildSurface::publish_surface(const Segment::ConstPtr &segment, bool is_new) -> SurfaceStamped::Ptr {
    ros::WallTime start = ros::WallTime::now();

    // Create output objects
    SurfaceStamped::Ptr surface = boost::make_shared<SurfaceStamped>();
    surface->header = segment->header;
    if (!is_new) surface->surface.id = segment->surface_id;
    surface->surface.model = segment->model;
    this->get_projected_cloud(segment, surface->surface.inliers);

    surface->surface.concave_hull = this->get_concave_hull(surface->surface.model, surface->surface.inliers);

    if (!this->verify_hull(surface->surface.concave_hull, surface->surface.model)) {
        if (is_new) {
            NODELET_ERROR_STREAM("Could not build new surface because the concave hull is invalid");
        } else {
            NODELET_ERROR_STREAM("Could not build surface " << surface->surface.id
                                 << " because the concave hull is invalid");
        }
//        return SurfaceStamped::Ptr();
    }

    (is_new ? this->pub_new_surface : this->pub_updated_surface).publish(surface);

    return surface;
}

surface_filters::BuildSurface::SurfaceMeshStamped::Ptr surface_filters::BuildSurface::publish_surface_mesh(
        const SurfaceStamped::ConstPtr &surface, bool is_new) {
    ros::WallTime start = ros::WallTime::now();

    // Create output objects
    SurfaceMeshStamped::Ptr surface_mesh = boost::make_shared<SurfaceMeshStamped>();
    surface_mesh->header = surface->header;
    surface_mesh->surface_mesh.id = surface->surface.id;
    this->get_3d_mesh(surface, surface_mesh->surface_mesh.surface_mesh);

    (is_new ? this->pub_new_surface_mesh : this->pub_updated_surface_mesh).publish(surface_mesh);

    auto num_triangles = surface_mesh->surface_mesh.surface_mesh.triangles.size();
//    NODELET_INFO_STREAM("[" << getName().c_str() << "::synchronized_input_callback] " << std::setprecision(3) <<
//                        "(" << (ros::WallTime::now() - start) << " sec, " << num_triangles << " triangles) " <<
//                        ") Published " << (is_new ? "new" : "updated") << " surface mesh");

    return surface_mesh;
}

void surface_filters::BuildSurface::config_callback(BuildSurfaceConfig &config,
                                                    uint32_t level __attribute__((unused))) {
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

using color_tup = std::array<float, 4>;

template <typename Iterator, typename PointT>
// Takes a container of pcl::Vertices
vis::Marker marker_from_vertices(const pcl::PointCloud<PointT> &cloud, const Iterator begin, const Iterator end,
                                 const int id, const std::string &name, const color_tup &color,
                                 const double width = 0.0025) {
    vis::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = cloud.header.frame_id;
    marker.ns = name;
    marker.id = id;
    marker.type = vis::Marker::LINE_LIST;
    marker.action = vis::Marker::MODIFY;
    // marker.pose not needed
    marker.scale.x = width; // scale.x controls the width of the line segments
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    for (auto it = begin; it < end; ++it) {
        marker.points.reserve(it->vertices.size());
        for (auto index = it->vertices.begin(); std::next(index) < it->vertices.end(); ++index) {
            const auto &point1 = cloud.at(*index);
            const auto &point2 = cloud.at(*(index + 1));
            geometry_msgs::Point pt1, pt2;
            std::tie(pt1.x, pt1.y, pt1.z) = std::tie(point1.x , point1.y, point1.z);
            std::tie(pt2.x, pt2.y, pt2.z) = std::tie(point2.x , point2.y, point2.z);
            marker.points.push_back(pt1);
            marker.points.push_back(pt2);
        }
    }

    return marker;
}

template <typename Iterator, typename PointT>
// Takes a container of point indices which are explicitly connected
vis::Marker marker_from_points(const pcl::PointCloud<PointT> &cloud, const Iterator begin, const Iterator end,
                                 const int id, const std::string &name, const color_tup &color,
                                 const double width = 0.025) {
    vis::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = cloud.header.frame_id;
    marker.ns = name;
    marker.id = id;
    marker.type = vis::Marker::LINE_LIST;
    marker.action = vis::Marker::MODIFY;
    // marker.pose not needed
    marker.scale.x = width; // scale.x controls the width of the line segments
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    const auto dist = std::distance(begin, end);
    std::cout << "Reserving " << dist << " elements" << std::endl;
    assert(dist <= marker.points.max_size());
    marker.points.reserve(dist);
    for (auto it = begin; it < end; ++it) {
        const auto &point1 = cloud.at(*it);
        geometry_msgs::Point pt1;
        std::tie(pt1.x, pt1.y, pt1.z) = std::tie(point1.x , point1.y, point1.z);
        marker.points.push_back(pt1);
    }

    return marker;
}

auto surface_filters::BuildSurface::get_concave_hull(const pcl::ModelCoefficients &model, const PointCloudIn &cloud) -> PolygonMesh {
    // Guaranteed properties of the meshes that come out of this function:
    // 1. The first polygon is the outer perimeter and the rest describe holes; no holes are outside the perimeter
    // 2. All boundary points are included in the hull cloud, even if they aren't used by any polygon
    // 3. Points which are not on the boundary are not in the hull cloud (the hull cloud is near minimal size)
    // 4. Every segment, including the one that implicitly closes a polygon, has length less than alpha
    // 5. Every polygon has at least three points
    // 6. There are no areas that are only connected by line segments
    // 7. Vertices are counterclockwise

    // 1. Explicitly vs implicitly closed is not guaranteed (this could be changed easily)
    // 2. There is no minimum thickness for any part of the hull

    PolygonMesh hull;
    hull.header = cloud.header;

    Triangulate tri(cloud, surfaces::tf_from_plane_model(model));

    // Triangulate from the triangle library, switches are:
    // z: makes indices start counting from zero, same convention as PCL
    // e: makes it output edges
    // D: tells it to do a Delauney triangulation
    // Q: (not used while debugging) don't print output
    tri.triangulate("zeDQ");

    const double alpha_squared = alpha_ * alpha_;
    auto edge_too_long_fn = [&](const Triangulate::Edge &edge) { return tri.edge_length_sqr(edge) > alpha_squared; };

    // ALPHA SHAPE: Build queriable map of non-removed triangles
    std::unordered_map<Triangulate::Edge, int, Triangulate::EdgeHash> num_adjacent_triangles;
    for (const Triangulate::Triangle &triangle : tri.triangles()) {
        const auto &edges = tri.triangle_edges(triangle);

        if (std::none_of(edges.begin(), edges.end(), edge_too_long_fn)) {
            for (const Triangulate::Edge &edge : edges) {
                num_adjacent_triangles[edge] += 1;
            }
        }
    }

    // ALPHA SHAPE: Build adjacency list of all boundary edges (edges that belong to exactly one non-deleted triangle)
    AdjacencyList point_adjacency;
    for (const Triangulate::Edge &edge : tri.edges()) {
        if (num_adjacent_triangles[edge] == 1) point_adjacency.add(edge);
    }

    // Function that takes the current point and its edge and returns the next point and its edge
    auto next_point = [&point_adjacency, &tri](AdjacencyList::iterator it) {
        auto this_point_i = it->first;
        auto next_point_i = it->second;

        bool both_removed = point_adjacency.remove(it);

        // Recompute neighbors
        auto neighbors = point_adjacency.neighbors(next_point_i);

        // Equal range returns iterators to the same element if there aren't matches, but not guaranteed to be the end
        // iterator. If not for this line then min_element would return neighbors.second when the caller expected .end()
        if (neighbors.first == neighbors.second) neighbors.first = neighbors.second = point_adjacency.end();

        const auto &origin = tri.get_point_eigen(next_point_i);
        const auto &ref_vector = tri.get_point_eigen(this_point_i) - origin;

        // Choose the rightmost element as the element with the largest dot product
        return std::min_element(neighbors.first, neighbors.second,
            [&tri, &origin, &ref_vector](const AdjacencyList::value_type &a, const AdjacencyList::value_type &b) {
                const auto &a_vector = tri.get_point_eigen(a.second) - origin;
                const auto &b_vector = tri.get_point_eigen(b.second) - origin;

                return ref_vector.dot(a_vector) < ref_vector.dot(b_vector);
            });
    };

    // ALPHA SHAPE: Make polygons out of edges
    while (!point_adjacency.empty()) {
        // Start from the lowest point (most negative y) going right as horizontally as possible (most positive x)
        // This guarantees counterclockwise order
        auto edge_it = std::min_element(point_adjacency.begin(), point_adjacency.end(),
            [&tri](const AdjacencyList::value_type &a, const AdjacencyList::value_type &b) {
                // If the points aren't the same, choose the one with the lowest y
                // If the points are the same, choose the neighbor with the largest x
                return (a.first != b.first) ? tri.get_y(a.first) < tri.get_y(b.first)
                                            : tri.get_x(a.second) > tri.get_x(b.second);
            });

        // Make a new polygon
        hull.polygons.emplace_back();

        do { // For every connected point
            // Add the point (iterator->first) to the polygon
            hull.polygons.back().vertices.push_back(edge_it->first);
            // Get the next point (also deletes the current point from the edge map)
            edge_it = next_point(edge_it);
        } while (edge_it != point_adjacency.end());
    }

    // ALPHA SHAPE: Find the largest polygon (by number of vertices) and move it to the front
    std::iter_swap(hull.polygons.begin(), std::min_element(hull.polygons.begin(), hull.polygons.end(),
                                                           [](const pcl::Vertices &a, const pcl::Vertices &b) {
        return a.vertices.size() > b.vertices.size();
    }));

    // Find polygons outside the outer perimeter and move them to the back of the list
    // (Modified erase-remove idiom to keep the to-be-erased elements valid during tri.cloud_from_polygons)
    const auto& perimeter_v = hull.polygons[0].vertices;
    const auto inside_perimeter = [&tri, &perimeter_v](const pcl::Vertices &v) -> bool {
        // Eliminate the influence of polygons sharing a vertex by taking 3 points and using the consensus
        // (If 2 or more vertices are inside the perimeter, then the polygon in inside the perimeter)
        // This "one" and "two" thing makes sure the minimum number of point in polygon checks are performed
        const bool one = tri.point_in_polygon(v.vertices[0], perimeter_v.begin(), perimeter_v.end());
        const bool two = tri.point_in_polygon(v.vertices[1], perimeter_v.begin(), perimeter_v.end());
        if (one && two) return true;
        if (one || two) return tri.point_in_polygon(v.vertices[2], perimeter_v.begin(), perimeter_v.end());
        return false;
    };
    const auto remove_start = std::partition(hull.polygons.begin() + 1, hull.polygons.end(),
                                             inside_perimeter);

    // Reindex and copy the cloud into hull as a pcl::PCLPointCloud2
    // This intentionally happens before the removal of polygons outside the outer perimeter because this
    // function is designed to keep all boundary points in the hull.cloud, even those not in the hull itself
    const auto new_hull_cloud = tri.cloud_from_polygons<PointIn>(hull.polygons);
    pcl::toPCLPointCloud2(new_hull_cloud, hull.cloud);

    // Remove the polygons
    hull.polygons.erase(remove_start, hull.polygons.end());

    return hull;
}

void surface_filters::BuildSurface::get_projected_cloud(const Segment::ConstPtr &segment,
                                                        PointCloudIn &proj_output) {
    // Uses shared_ptrs' alaiasing constructor to get shared pointers to the inliers cloud and to the model coefficients
    // that cooperate with the shared pointer to the segment
    proj_.setInputCloud(PointCloudIn::ConstPtr(segment, &segment->inliers));
    proj_.setModelCoefficients(pcl::ModelCoefficients::ConstPtr(segment, &segment->model));
    proj_.filter(proj_output);
}

void surface_filters::BuildSurface::get_3d_mesh(const SurfaceStamped::ConstPtr &surface, shape_msgs::Mesh &output_trimesh) {
    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(surface->surface.concave_hull.cloud, hull_cloud);
    assert(hull_cloud.size() == surface->surface.concave_hull.cloud.width * surface->surface.concave_hull.cloud.height);

    std::vector<double> points_2d;
    std::vector<int> segment_list;
    std::vector<double> holes;

    this->get_triangulation_vertices(surface->surface.model, hull_cloud, output_trimesh, points_2d);
    assert(points_2d.size() == hull_cloud.size() * 2);
    this->get_triangulation_segments(surface->surface.concave_hull, points_2d, segment_list, holes);
    assert(points_2d.size() == hull_cloud.size() * 2);
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
    tri_out.pointlist = NULL; // NULL tells Triangulate to allocate the memory for it
    tri_out.trianglelist = NULL; // NULL tells Triangulate to allocate the memory for it

    // Triangulate from the triangle library, switches are:
    // z: makes indices start counting from zero, same convention as PCL
    // p: tells it to triangulate a polygon
    // P: don't output the segments to save memory (if this changes, initialize tri_out.segmentlist)
    // B: don't output the boundary markers to save memory (if this changes, initialize tri_out.segmentmarkerlist)
    // Q: (not used while debugging) don't print output
    // IMPORTANT: If the flags change, make sure to change tri_in and tri_out accordingly
    triangulate("zpPBQ", &tri_in, &tri_out, NULL);
    assert(tri_out.numberofcorners == 3);
    assert(tri_out.pointlist != NULL);
    assert(tri_out.trianglelist != NULL);

    assert(tri_out.numberofpoints > 0);
    assert(tri_in.numberofpoints > 0);
    const uint32_t &num_points_out = tri_out.numberofpoints;
    const uint32_t &num_points_in = tri_in.numberofpoints;

    assert(num_points_out >= num_points_in);

    // Output gets 2 triangles per tri_out triangle (one for the top layer and one for the bottom), plus 2 triangles
    // per edge segment (to join the layers). Note that segment_list is already (2 * # of edge segments).
    output_trimesh.triangles.resize(static_cast<std::size_t>(tri_out.numberoftriangles) * 2 + segment_list.size());

    assert(output_trimesh.vertices.size() % 2 == 0);
    int cloud_size = static_cast<int>(output_trimesh.vertices.size()) / 2;
    std::size_t count = 0;

    std::vector<uint32_t> additional_points_mapping;
    if (num_points_out > num_points_in) {
        auto make_range = [](double *arr, uint32_t size, uint32_t offset) {
            return boost::adaptors::stride(boost::make_iterator_range(arr + offset, arr + 2 * size), 2);
        };

        const auto existing_x = make_range(tri_in.pointlist, num_points_in, 0);
        const auto existing_y = make_range(tri_in.pointlist, num_points_in, 1);
        const auto additional_x = make_range(tri_out.pointlist + num_points_in * 2, num_points_out - num_points_in, 0);
        const auto additional_y = make_range(tri_out.pointlist + num_points_in * 2, num_points_out - num_points_in, 1);

        std::vector<PointIn> existing_points;
        std::transform(existing_x.begin(), existing_x.end(), existing_y.begin(), std::back_inserter(existing_points),
                       [](const double &existing_pt_x, const double &existing_pt_y) -> PointIn {
                           return {static_cast<float>(existing_pt_x), static_cast<float>(existing_pt_y), 0};
                       }
        );
        std::transform(additional_x.begin(), additional_x.end(), additional_y.begin(),
                       std::back_inserter(additional_points_mapping),
                       [&existing_points](const double &additional_pt_x, const double &additional_pt_y) -> long {
                           PointIn overflow_pt(static_cast<float>(additional_pt_x),
                                               static_cast<float>(additional_pt_y), 0);

                           return std::distance(existing_points.begin(),
                                std::min_element(existing_points.begin(), existing_points.end(),
                                                 [&overflow_pt](const PointIn& pt_a, const PointIn& pt_b) {
                                                     return pcl::squaredEuclideanDistance(overflow_pt, pt_a) <
                                                             pcl::squaredEuclideanDistance(overflow_pt, pt_b);
                    }));
                }
        );
    }
    assert(additional_points_mapping.size() == num_points_out - num_points_in);

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

        // In some circumstances (including an hourglass shape), triangle adds duplicates of the points, and if it does
        // then tri_out.trianglelist will contain out-of-bounds indices. This detects them and replaces them with the
        // closest point in the original list
        if (num_points_out > num_points_in) {
            for (std::size_t idx = 0; idx < output_trimesh.triangles[i].vertex_indices.size(); idx++) {
                const uint32_t &old_val = output_trimesh.triangles[i].vertex_indices[idx];
                if (old_val >= num_points_in) {
                    // Use 'at' because it has bounds checking
                    const uint32_t &new_val = additional_points_mapping.at(old_val - num_points_in);

                    output_trimesh.triangles[i].vertex_indices[idx] = new_val;
                    output_trimesh.triangles[offset_i].vertex_indices[idx] = new_val + cloud_size;

//                    NODELET_DEBUG_STREAM("Triangulate at index " << i << " coordinate " << idx << " was out of bounds (" <<
//                                         old_val << "), replaced with closest point: " << new_val);
                }
            }
        }

        count += 2;
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

            count += 2;
        }
        offset += polysize * 2;
    }

    assert(count == output_trimesh.triangles.size());
    trifree(tri_out.pointlist);
    trifree(tri_out.trianglelist);
}

bool surface_filters::BuildSurface::verify_hull(const PolygonMesh &hull, const pcl::ModelCoefficients &model) const {
    // Verifies three assumptions:
    // 1. None of the lines share endpoints unless they are adjacent (even across polygons)
    // 2. None of the lines intersect (even across polygons)
    // 3. The first polygon is the outer perimeter and all the rest are holes

    return /* verify_hull_unique_vertices(hull) && */ verify_hull_no_self_intersection(hull, model) &&
           /* verify_hull_perimeter_holes(hull) && */ verify_hull_segment_lengths(hull);
}

bool surface_filters::BuildSurface::verify_hull_unique_vertices(const PolygonMesh &hull) const {
    // Verifies none of the lines share endpoints unless they are adjacent (even across polygons)
    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(hull.cloud, hull_cloud);

    // Make a vector representing every point to check whether it has been used or not, defaulting to false
    std::vector<bool> idx_used(hull.cloud.height * hull.cloud.width, false);

    for (const pcl::Vertices &vertices : hull.polygons) {
        // It is acceptable, but not required, for the front to be equal to the back (explicitly closed polygon)
        // If the last element is equal to the first, ignore it when iterating
        bool ignore_last = (vertices.vertices.front() == vertices.vertices.back());

        std::size_t count = 0;

        // The make-iterator-range ignores the last element if ignore_last is true
        for (const uint32_t &vertex : boost::make_iterator_range(vertices.vertices, 0, ignore_last ? -1 : 0)) {
            if (vertex >= idx_used.size()) {
                NODELET_ERROR("Hull invalid -- index %u out of bounds", vertex);
                return false;
            } else if (idx_used[vertex]) {
                NODELET_ERROR("Hull invalid -- index %u used more than once", vertex);
                return false;
            } else {
                idx_used[vertex] = true;
            }
            count++;
        }

        for (auto pt1 = hull_cloud.begin(); pt1 < hull_cloud.end(); ++pt1) {
            // It's fine for points to be duplicated if they're never used
            if (!idx_used[std::distance(hull_cloud.begin(), pt1)]) continue;

            if (!pcl_isfinite(pt1->x) || !pcl_isfinite(pt1->y) || !pcl_isfinite(pt1->z)) {
                NODELET_ERROR_STREAM("Hull invalid -- point " << std::distance(hull_cloud.begin(), pt1) << " is NaN");
            }
            for (auto pt2 = pt1 + 1; pt2 < hull_cloud.end(); ++pt2) {
                if (pcl::squaredEuclideanDistance(*pt1, *pt2) < 1e-10) {
                    NODELET_ERROR_STREAM("Hull invalid -- point " << std::distance(hull_cloud.begin(), pt1) << " is at " <<
                                         " virtually the same location as " << std::distance(hull_cloud.begin(), pt2));
                    return false;
                }
            }
        }

        assert( (!ignore_last && count == vertices.vertices.size()) ||
                (ignore_last && count == vertices.vertices.size() - 1) );
    }

    long unused_indices = std::count(idx_used.begin(), idx_used.end(), false);
    if (unused_indices > 0) {
        NODELET_WARN("Hull has %lu unused indices", unused_indices);
    }

    return true;
}

bool surface_filters::BuildSurface::verify_hull_no_self_intersection(const PolygonMesh &hull,
                                                                     const pcl::ModelCoefficients &model) const {
    // Verifies none of the lines intersect (even across polygons)

    // Get and fill a points_2d vector of projected points
    std::vector<double> points_2d;

    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(hull.cloud, hull_cloud);
    shape_msgs::Mesh dummy;
    this->get_triangulation_vertices(model, hull_cloud, dummy, points_2d);

//    std::vector<std::pair<geeksforgeeks::Point, geeksforgeeks::Point>> segments;
//
//    for (const pcl::Vertices &vertices : hull.polygons) {
//        segments.reserve(segments.size() + vertices.vertices.size());
//
//        // It is acceptable, but not required, for the front to be equal to the back (explicitly closed polygon)
//        // If the last element is equal to the first, ignore it when iterating and treat polygon as implicitly closed
//        std::size_t last_index = vertices.vertices.size();
//        if (vertices.vertices.front() == vertices.vertices.back()) last_index -= 1;
//
//        // Iterate over all but the last element (or the last 2 if it was explicitly closed) to avoid considering
//        // the final line segment (it can never intersect unless at least one other does, and it's hard to handle)
//        for (std::size_t i = 0; i + 1 < last_index; i++) {
//            // i + 1 is safe because i never gets to the last item in the vector
//            decltype(segments)::value_type next_seg({tri_x(points_2d, vertices.vertices[i]),
//                                                     tri_y(points_2d, vertices.vertices[i])},
//                                                    {tri_x(points_2d, vertices.vertices[i + 1]),
//                                                     tri_y(points_2d, vertices.vertices[i + 1])});
//
//            // Make a range that skips the previous segment unless this is the first segment of the current polygon
//            for (const auto &segment : boost::make_iterator_range(segments, 0, (i == 0) ? 0 : -1)) {
//                if ((segment.first == next_seg.first || segment.first == next_seg.second) ||
//                        (segment.second == next_seg.first || segment.second == next_seg.second) ) {
//                    // In this case they share an endpoint and another self-intersection test must be used.
//                    // I don't know how to write that test
////                    NODELET_WARN_STREAM("Segments share an endpoint and won't be checked for self-intersection");
//                } else if (geeksforgeeks::doIntersect(segment.first, segment.second, next_seg.first, next_seg.second)) {
//                    NODELET_ERROR_STREAM("Hull invalid -- self-intersection (points " << segment.first << ", "
//                                         << segment.second << ", " << next_seg.first << ", " << next_seg.second);
//                    return false;
//                }
//            }
//
//            segments.push_back(std::move(next_seg));
//        }
//    }

    return true;
}

bool surface_filters::BuildSurface::verify_hull_perimeter_holes(const PolygonMesh &hull) const {
    // Verifies that the first polygon is the outer perimeter and all the rest are holes

    if (hull.polygons.size() <= 1) {
        // Then this can't possibly be false
        return true;
    }

    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(hull.cloud, hull_cloud);

    std::vector<PointCloudIn> hole_clouds;
    hole_clouds.reserve(hull.polygons.size() - 1);

    auto &perimeter_vertices = hull.polygons[0].vertices;
    PointCloudIn perimeter(hull_cloud, std::vector<int>(perimeter_vertices.begin(), perimeter_vertices.end()));

    for (const pcl::Vertices &vertices : boost::make_iterator_range(hull.polygons, 1, 0)) {
        PointCloudIn cloud(hull_cloud, std::vector<int>(vertices.vertices.begin(), vertices.vertices.end()));

        // Make sure every member of this cloud is within the perimeter
        for (const PointIn &point : cloud.points) {
            if (!pcl::isPointIn2DPolygon(point, perimeter)) {
                NODELET_ERROR("Hull invalid -- there are points outside the perimeter (first polygon)");
                return false;
            }
        }

        // Make sure no member of this cloud is within another hole
        for (const PointCloudIn &hole_cloud : hole_clouds) {
            for (const PointIn &point : cloud.points) {
                if (pcl::isPointIn2DPolygon(point, hole_cloud)) {
                    NODELET_ERROR("Hull invalid -- there are points inside a hole");
                    return false;
                }
            }
        }

        hole_clouds.push_back(cloud);
    }

    return true;
}

bool surface_filters::BuildSurface::verify_hull_segment_lengths(const PolygonMesh &hull) const {
    // Verifies that none of the segments have zero (or near-zero) length

    PointCloudIn hull_cloud;
    pcl::fromPCLPointCloud2(hull.cloud, hull_cloud);

    for (const pcl::Vertices &vertices : hull.polygons) {
        // It is acceptable, but not required, for the front to be equal to the back (explicitly closed polygon)
        // If the last element is equal to the first, ignore it when iterating and treat polygon as implicitly closed
        std::size_t last_index = vertices.vertices.size();
        if (vertices.vertices.front() == vertices.vertices.back()) last_index -= 1;

        // Iterate over all but the last element (or the last 2 if it was explicitly closed) to avoid considering
        // the final line segment (it can never intersect unless at least one other does, and it's hard to handle)
        for (std::size_t i = 0; i + 1 < last_index; i++) {
            // i + 1 is safe because i never gets to the last item in the vector
            auto segment_length = pcl::euclideanDistance(hull_cloud.at(vertices.vertices.at(i)),
                                                         hull_cloud.at(vertices.vertices.at(i + 1)));
            if (segment_length < 1e-10) {
                NODELET_ERROR("Hull invalid -- there is a zero-length segment");
                return false;
            }

            if (segment_length > alpha_ * 2) {
                NODELET_ERROR_STREAM("Hull invalid -- there is a segment with length greater than 2 * " << alpha_ << " (length is " << segment_length << ")");
                return false;
            }
        }
    }

    return true;
}


PLUGINLIB_EXPORT_CLASS(surface_filters::BuildSurface, nodelet::Nodelet)
