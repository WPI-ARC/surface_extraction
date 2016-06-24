//
// Created by will on 6/22/16.
//

#include "build_surface/BuildSurface.h"

// std / Boost include

// Surface types includes
#include <surface_types/Surface.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>

// Utility includes
#include <surface_utils/smart_ptr.hpp>
#include <surface_utils/pcl_utils.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

// Get the source vertex of an edge (templated so it works on both types of iterators)
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_source(T &it) {
    return it->first->vertex(it->first->cw(it->second));
};

// Get target vertex of an edge (templated so it works on both types of iterators)
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_target(T &it) {
    return it->first->vertex(it->first->ccw(it->second));
};

template <typename T>
inline bool is_in(const std::set<std::pair<uint32_t, uint32_t>> &set, T &it) {
    // Needs initializer-list syntax so it will return a non-const pair
    return set.find(std::minmax({edge_source(it)->info().pcl_index, edge_target(it)->info().pcl_index})) != set.end();
};

template <typename T>
inline bool add_to(std::set<std::pair<uint32_t, uint32_t>> &set, T &it) {
    // Needs initializer-list syntax so it will return a non-const pair
    return set.insert(std::minmax({edge_source(it)->info().pcl_index, edge_target(it)->info().pcl_index})).second;
};

BuildSurface::BuildSurface(double perpendicular_distance, double alpha, float extrude_distance)
    : perpendicular_distance_(perpendicular_distance), alpha_(alpha), extrude_distance_(extrude_distance), next_id_(1),
      color_gen_(0.5, 0.99) {}

void BuildSurface::build_updated_surface(const Surface &old_surface,
                                         const std::function<void(BuildSurface::Surface)> callback) {
    pcl::ScopeTime st("BuildSurface::build_updated_surface");

    Surface updated_surface;

    // ID and color don't change
    updated_surface.id = old_surface.id;
    updated_surface.color = old_surface.color;

    // Inliers was already updated
    updated_surface.inliers = old_surface.inliers;

    // The rest needs to be computed
    //    updated_surface.model = find_model_for_inliers(updated_surface.inliers, old_surface.model);
    //    updated_surface.pose = adjust_pose_to_model(old_surface.pose, updated_surface.model);
    //    auto p = ProgressListener();
    //    std::tie(updated_surface.boundary, updated_surface.polygons) =
    //        find_boundary_and_polygons(updated_surface.inliers, updated_surface.pose, p);
    //    updated_surface.polygons = simplify_polygons(<#initializer #>, updated_surface.polygons);
    //    updated_surface.mesh = create_trimesh(updated_surface.boundary, updated_surface.polygons);

    callback(updated_surface);
}

void BuildSurface::build_new_surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3d pose,
                                     ProgressListener &p, std::function<void(Surface)> callback) {
    pcl::ScopeTime st("BuildSurface::build_new_surface");
    Surface new_surface;

    // ID and color are set using class attributes
    // NOTE if this function is parallelized, these need to be put behind a memory barrier
    new_surface.id = next_id_++;
    new_surface.color.a = 1;
    std::tie(new_surface.color.r, new_surface.color.g, new_surface.color.b) = color_gen_.rgb();

    // inliers, model, and pose are given
    new_surface.inliers = inliers;
    new_surface.model = model;
    new_surface.pose = pose;
    auto pose_float = new_surface.pose.cast<float>();

    p.pose("surface_pose", pose);

    // boundary, polygons, and mesh are computed with CGAL
    auto cgal_points = get_cgal_2d_points(new_surface.inliers, pose_float);
    std::tie(new_surface.boundary, new_surface.polygons) =
        find_boundary_and_polygons(new_surface.inliers, cgal_points, pose_float, p);
    auto triangulation = get_simplified_triangulation(cgal_points, new_surface.polygons);

    new_surface.polygons = simplify_polygons(triangulation);
    p.polygons("polygons", new_surface);
    new_surface.mesh = create_trimesh(triangulation, pose_float);
    p.mesh("mesh", new_surface);

    callback(new_surface);
}

pcl::ModelCoefficients BuildSurface::find_model_for_inliers(const PointCloud &inliers) {
    PointCloud::ConstPtr p(&inliers, null_deleter());
    pcl::SampleConsensusModelPlane<Point> sacmodel(std::move(p));

    Eigen::VectorXf coefficients;
    sacmodel.computeModelCoefficients(surfaces_pcl_utils::all_indices(inliers), coefficients);

    auto points_outside = inliers.size() - sacmodel.countWithinDistance(coefficients, perpendicular_distance_);

    if (points_outside > 0.05 * inliers.size()) {
        ROS_ERROR_STREAM("New model excludes " << static_cast<double>(points_outside) / inliers.size() * 100
                                               << "% of points (" << points_outside << " / " << inliers.size());
        return pcl::ModelCoefficients();
    } else if (points_outside > 0) {
        ROS_WARN_STREAM("New model excludes " << static_cast<double>(points_outside) / inliers.size() * 100
                                              << "% of points (" << points_outside << " / " << inliers.size());
    }

    pcl::ModelCoefficients ret;
    ret.header = inliers.header;
    ret.values = {coefficients[0], coefficients[1], coefficients[2], coefficients[3]};

    return ret;
}

pcl::ModelCoefficients BuildSurface::find_model_for_inliers(const PointCloud &cloud,
                                                            const pcl::ModelCoefficients &prev_model) {
    auto new_model = find_model_for_inliers(cloud);

    // STL way to get dot product (probably unrolled under -03 ?)
    auto d = std::inner_product(new_model.values.begin(), new_model.values.begin() + 3, prev_model.values.begin(), 0);

    // If dot product is less than 0, the vectors (which represent normal vectors) were more than 90deg apart
    if (d < 0) {
        // Note reference type of loop variable
        for (auto &val : new_model.values)
            val *= -1;
    }

    return new_model;
}

Eigen::Affine3d BuildSurface::adjust_pose_to_model(Eigen::Affine3d old_pose, pcl::ModelCoefficients model) {
    ROS_WARN_STREAM("adjust_pose_to_model has not been tested yet");
    // Promote all the floats coming from model to doubles
    const auto plane_normal = Eigen::Map<Eigen::Vector3f>(model.values.data()).cast<double>();
    const auto plane_distance = static_cast<double>(model.values[3]);

    // Projection vector obtained by multiplying the point-to-plane distance (from the dot product) by the plane_normal
    const auto projection_translate = (old_pose.translation().dot(plane_normal) + plane_distance) * plane_normal;
    ROS_DEBUG_STREAM("translating by " << projection_translate);

    // Using (X^ + Y^) as the reference point to compute the rotation
    const auto p = Eigen::Vector3d::UnitX() + Eigen::Vector3d::UnitY();

    // Project p onto the plane
    const auto p_ = (p.dot(plane_normal) + plane_distance) * plane_normal;

    const auto rotation_angle = std::acos(p.dot(p_) / (p.norm() * p_.norm()));
    const auto rotation_axis = p.cross(p_);
    ROS_DEBUG_STREAM("rotating by " << rotation_angle << " radians about axis " << rotation_axis);

    return old_pose.translate(projection_translate).rotate(Eigen::AngleAxisd(rotation_angle, rotation_axis));
}

std::vector<CustomPoint> BuildSurface::get_cgal_2d_points(const PointCloud &cloud, const Eigen::Affine3f &transform) {
    // Make a transformed cloud
    auto cloud_transformed = boost::make_shared<PointCloud>(cloud);
    pcl::transformPointCloud(cloud, *cloud_transformed, transform.inverse());

    // Copy the points into a CGAL data structure and simultaneously project
    std::vector<CustomPoint> cgal_points;
    const auto proj_warn_threshold = perpendicular_distance_ * 2;
    for (std::size_t i = 0; i < cloud_transformed->size(); i++) {
        auto pt = cloud_transformed->points[i];
        if (pt.z > proj_warn_threshold || pt.z < -proj_warn_threshold) {
            ROS_WARN_STREAM("Projected point is more than " << proj_warn_threshold << "m from zero (" << pt.z << "m)");
        }
        cgal_points.emplace_back(CGALPoint(pt.x, pt.y), CustomPointInfo(static_cast<uint32_t>(i)));
    }

    return cgal_points;
}

std::tuple<BuildSurface::PointCloud, std::vector<pcl::Vertices>>
BuildSurface::find_boundary_and_polygons(const PointCloud &cloud, const std::vector<CustomPoint> &cgal_points,
                                         const Eigen::Affine3f &transform, ProgressListener &p) {

    // Create a regularized alpha shape with the alpha set to the user's provided alpha
    Alpha_shape_2 shape(cgal_points.begin(), cgal_points.end(), alpha_, Alpha_shape_2::REGULARIZED);

    // Find the boundary indices
    PointCloud boundary_points;
    for (auto it = shape.alpha_shape_vertices_begin(); it != shape.alpha_shape_vertices_end(); ++it) {
        if (shape.classify(*it) == Alpha_shape_2::REGULAR) {
            Point p((*it)->point().x(), (*it)->point().y(), 0);
            boundary_points.push_back(pcl::transformPoint(p, transform));
        } else {
            // TODO If this assert isn't triggered after testing, remove the conditional
            assert(false && "Assumption failed -- alpha shape contains non-regular points");
        }
    }

    std::set<std::pair<uint32_t, uint32_t>> visited;

    // Find the polygons
    std::vector<pcl::Vertices> polygons;
    for (auto start_edge = shape.alpha_shape_edges_begin(); start_edge != shape.alpha_shape_edges_end(); ++start_edge) {
        // start_edge is an iterator to a pair <Face_handle, int>, where the int is an edge of the face
        if (!is_in(visited, start_edge)) {
            pcl::Vertices polygon;

            // The start edge will be the implicitly closed edge. Start the search at its target and iterate until
            // the vector ends with its source.
            auto polygon_last_index = edge_source(start_edge)->info().pcl_index;
            Alpha_shape_2::Edge_circulator current_edge =
                shape.incident_edges(edge_target(start_edge), start_edge->first);

            // Add vertices until the polygon is implicitly closed
            // (uses a do-while because .back() is invalid until at least one loop iteration)
            auto prev_source = -1;
            do {
                Alpha_shape_2::Edge_circulator start_current_edge = current_edge;
                // Find the first regular (boundary) incident edge which hasn't been visited, going counterclockwise
                // Note an edge has only been visited if its source AND target have been visited
                while (is_in(visited, current_edge) || shape.classify(*current_edge) != Alpha_shape_2::REGULAR) {
                    ++current_edge;
                    assert(edge_target(current_edge)->info().pcl_index != prev_source &&
                           "Went into an infinite loop while finding the next edge");
                }

                // Add this edge's start vertex (aka source)
                polygon.vertices.push_back(edge_source(current_edge)->info().pcl_index);
                add_to(visited, current_edge);

                prev_source = edge_source(current_edge)->info().pcl_index;

                // Get a circulator of all edges incident to the target vertex
                current_edge = shape.incident_edges(edge_target(current_edge), current_edge->first);
            } while (polygon.vertices.back() != polygon_last_index);
            // Need to add implicit edge to the visited list (note: DON'T use current_edge, it hasn't been filtered yet)
            visited.insert(std::minmax({polygon.vertices.front(), polygon.vertices.back()}));

            polygons.push_back(polygon);
        }
    }

    return std::make_tuple(boundary_points, polygons);
}

CT BuildSurface::get_simplified_triangulation(const std::vector<CustomPoint> &cgal_points,
                                              const std::vector<pcl::Vertices> &polygons) const {
    CT ct;
    for (auto &pcl_polygon : polygons) {
        auto i_to_pt = [&cgal_points](uint32_t i) { return cgal_points[i].first; };
        // I'm very smudge about correctly using tranform iterators
        // IMPORTANT: The last boolean tells ct to close the polygon (it's implicitly closed)
        // This overload of insert_constraint inserts a closed (if closed=true) polyline
        auto cid =
            ct.insert_constraint(boost::iterators::make_transform_iterator(pcl_polygon.vertices.begin(), i_to_pt),
                                 boost::iterators::make_transform_iterator(pcl_polygon.vertices.end(), i_to_pt), true);

        // Can't pass in info to insert_constraint for some reason, so set it after
        auto it = ct.vertices_in_constraint_begin(cid);
        for (auto &index : pcl_polygon.vertices) {
            //            assert(it != ct.vertices_in_constraint_end(cid));
            //            assert((*it)->point() == cgal_points[index].first);
            (*it)->info() = cgal_points[index].second;
            ++it;
        }
    }

    // TODO should alpha be re-used for this?
    simplify(ct, SimplificationCost(), SimplificationStop(alpha_));
    return ct;
}

std::vector<pcl::Vertices> BuildSurface::simplify_polygons(const CT &ct) {
    std::vector<pcl::Vertices> simplified;
    for (auto cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
        pcl::Vertices polygon;
        for (auto vit = ct.vertices_in_constraint_begin(*cit); vit != ct.vertices_in_constraint_end(*cit); ++vit) {
            polygon.vertices.push_back((*vit)->info().pcl_index);
        }
        simplified.push_back(polygon);
    }

    return simplified;
}

void mark_domains(CT &ct, CT::Face_handle start, int index, std::list<CT::Edge> &border) {
    if (start->info().nesting_level != -1) return;
    std::list<CT::Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()) {
        CT::Face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1) {
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++) {
                CT::Edge e(fh, i);
                CT::Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1) {
                    if (ct.is_constrained(e))
                        border.push_back(e);
                    else
                        queue.push_back(n);
                }
            }
        }
    }
}
// explore set of facets connected with non constrained edges,
// and attribute to each such set a nesting level.
// We start from facets incident to the infinite vertex, with a nesting
// level of 0. Then we recursively consider the non-explored facets incident
// to constrained edges bounding the former set and increase the nesting level by 1.
// Facets in the domain are those with an odd nesting level.
void mark_domains(CT &cdt) {
    for (auto it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
        it->info().nesting_level = -1;
    }
    std::list<CT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CT::Edge e = border.front();
        border.pop_front();
        CT::Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1) {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}

shape_msgs::Mesh BuildSurface::create_trimesh(CT ct, const Eigen::Affine3f &transform) {
    // Identify all the faces in the polygon (aka domain)
    // Shamelessly lifted from one of the examples in the CGAL manual
    mark_domains(ct);

    shape_msgs::Mesh mesh;
    // Add the top (true) and bottom (false) faces
    add_mesh_face(mesh, transform, ct, true);
    add_mesh_face(mesh, transform, ct, false);

    // TODO Sides

    return mesh;
}

void BuildSurface::add_mesh_face(shape_msgs::Mesh &mesh, const Eigen::Affine3f &transform, const CT &ct,
                                 bool bottom) const {
    // Re-use visited to indicate whether this point has been added to the mesh
    for (auto it = ct.all_vertices_begin(); it != ct.all_vertices_end(); ++it) {
        it->info().visited = false;
    }

    // Add the points and triangles for the top surface
    for (auto it = ct.finite_faces_begin(); it != ct.finite_faces_end(); ++it) {
        if (it->info().in_domain()) {
            shape_msgs::MeshTriangle mesh_triangle;
            for (int loop_i = 0; loop_i < 3; loop_i++) {
                auto i = bottom ? (2 - loop_i) : loop_i; // Flip triangles when it's the bottom
                if (!it->vertex(i)->info().visited) {    // Then it's not yet in the mesh meshage
                    it->vertex(i)->info().pcl_index = static_cast<uint32_t>(mesh.vertices.size());
                    it->vertex(i)->info().visited = true;
                    // Get the 3d point and add it to the mesh
                    Point p2d(it->vertex(i)->point().x(), it->vertex(i)->point().y(), bottom ? -extrude_distance_ : 0);
                    Point p3d = transformPoint(p2d, transform);
                    geometry_msgs::Point p;
                    p.x = p3d.x;
                    p.y = p3d.y;
                    p.z = p3d.z;
                    mesh.vertices.push_back(p);
                }

                mesh_triangle.vertex_indices[i] = it->vertex(i)->info().pcl_index;
            }
            mesh.triangles.push_back(mesh_triangle);
        }
    }
}
