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

    // ros console escapes my backslashes
    ROS_INFO("%c[38;2;%d;%d;%dmBuilding updated surface: %d", 0x1B, static_cast<int>(old_surface.color.r * 256),
             static_cast<int>(old_surface.color.g * 256), static_cast<int>(old_surface.color.b * 256), old_surface.id);

    // Inliers was already updated
    updated_surface.inliers = old_surface.inliers;

    // The rest needs to be computed
    updated_surface.model = find_model_for_inliers(updated_surface.inliers, old_surface.model);
    updated_surface.pose = adjust_pose_to_model(old_surface.pose, updated_surface.model);
    auto pose_float = updated_surface.pose.cast<float>();

    auto p = ProgressListener();
    auto cgal_points = get_cgal_2d_points(updated_surface.inliers, pose_float);
    std::tie(updated_surface.boundary, updated_surface.polygons) =
        find_boundary_and_polygons(updated_surface.inliers, cgal_points, pose_float, p);
    auto triangulation = get_simplified_triangulation(cgal_points, updated_surface.polygons);

    auto un_simplified_polygons = updated_surface.polygons;

    updated_surface.polygons = simplify_polygons(triangulation);

    // Ensure the polygon simplification only removed vertices, didn't reorder the ones that weren't removed
    for (std::size_t i = 0; i < updated_surface.polygons.size(); i++) {
        auto simplified_vertices = updated_surface.polygons[i].vertices;
        auto unsimplified_vertices = un_simplified_polygons[i].vertices;

        auto curr_pos = unsimplified_vertices.begin();
        for (auto &index : simplified_vertices) {
            // Find the next occurrence of the simplified index in the unsimplified list
            curr_pos = std::find(curr_pos, unsimplified_vertices.end(), index);
            if (curr_pos == unsimplified_vertices.end()) {
                ROS_ERROR("Got changed vertices order: ");
                std::copy(unsimplified_vertices.begin(), unsimplified_vertices.end(),
                          std::ostream_iterator<uint32_t>(std::cout, ", "));
                std::cout << std::endl;
                std::copy(simplified_vertices.begin(), simplified_vertices.end(),
                          std::ostream_iterator<uint32_t>(std::cout, ", "));
                std::cout << std::endl;
            }
            assert(curr_pos != unsimplified_vertices.end() && "Simplifying polygons changed the vertices' order");
        }
    }

    p.polygons("polygons", updated_surface);
    updated_surface.mesh = create_trimesh(triangulation, pose_float);
    p.mesh("mesh", updated_surface);

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

    // ros console escapes my backslashes
    ROS_INFO("%c[38;2;%d;%d;%dmBuilding new surface: %d", 0x1B, static_cast<int>(new_surface.color.r * 256),
             static_cast<int>(new_surface.color.g * 256), static_cast<int>(new_surface.color.b * 256), new_surface.id);

    // inliers, model, and pose are given
    new_surface.inliers = inliers;
    new_surface.model = model;
    new_surface.pose = pose;
    auto pose_float = new_surface.pose.cast<float>();

    p.pose("surface_pose", new_surface);
    p.plane_normal("surface_normal", new_surface);

    // boundary, polygons, and mesh are computed with CGAL
    auto cgal_points = get_cgal_2d_points(new_surface.inliers, pose_float);
    std::tie(new_surface.boundary, new_surface.polygons) =
        find_boundary_and_polygons(new_surface.inliers, cgal_points, pose_float, p);
    auto triangulation = get_simplified_triangulation(cgal_points, new_surface.polygons);
    p.points<Point>("boundary_points", new_surface.boundary.makeShared());

    new_surface.polygons = simplify_polygons(triangulation);
    p.polygons("polygons", new_surface);
    new_surface.mesh = create_trimesh(triangulation, pose_float);
    p.mesh("mesh", new_surface);

    callback(new_surface);
}

pcl::ModelCoefficients BuildSurface::optimize_model_for_inliers(const PointCloud &inliers,
                                                                const pcl::ModelCoefficients &model) {
    PointCloud::ConstPtr p(&inliers, null_deleter());
    pcl::SampleConsensusModelPlane<Point> sacmodel(std::move(p));

    Eigen::VectorXf coefficients(4), old_co(4);
    old_co << model.values[0], model.values[1], model.values[2], model.values[3];
    sacmodel.optimizeModelCoefficients(surfaces_pcl_utils::all_indices(inliers), old_co, coefficients);

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
    auto new_model = optimize_model_for_inliers(cloud, prev_model);

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
    auto boundary_points_flat = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (auto it = shape.alpha_shape_vertices_begin(); it != shape.alpha_shape_vertices_end(); ++it) {
        if (shape.classify(*it) == Alpha_shape_2::REGULAR) {
            Point pt((*it)->point().x(), (*it)->point().y(), 0);
            boundary_points.push_back(pcl::transformPoint(pt, transform));
            auto color_tuple = color_gen_.repeat_rgb(); // Should get the same color as the surface
            pcl::PointXYZRGB cpt(static_cast<uint8_t>(std::get<0>(color_tuple) * 255),
                                 static_cast<uint8_t>(std::get<1>(color_tuple) * 255),
                                 static_cast<uint8_t>(std::get<2>(color_tuple) * 255));
            cpt.x = (*it)->point().x();
            cpt.y = (*it)->point().y();
            cpt.z = 0;
            boundary_points_flat->push_back(cpt);
        } else {
            // TODO If this assert isn't triggered after testing, remove the conditional
            assert(false && "Assumption failed -- alpha shape contains non-regular points");
        }
    }

    p.points<pcl::PointXYZRGB>("boundary_points_projected", boundary_points_flat);

    std::set<std::pair<uint32_t, uint32_t>> visited;

    // Find the polygons
    std::vector<pcl::Vertices> polygons;
    std::size_t largest_polygon_id = 0;
    double largest_polygon_area_x2 = 0;
    for (auto start_edge = shape.alpha_shape_edges_begin(); start_edge != shape.alpha_shape_edges_end(); ++start_edge) {
        assert(shape.classify(*start_edge) == Alpha_shape_2::REGULAR && "Polygon gave a non-regular edge");
        // start_edge is an iterator to a pair <Face_handle, int>, where the int is an edge of the face
        if (!is_in(visited, start_edge)) {
            pcl::Vertices polygon;

            // The start edge will be the implicitly closed edge. Start the search at its target and iterate until
            // the vector ends with its source.
            auto polygon_last_index = edge_source(start_edge)->info().pcl_index;
            Alpha_shape_2::Edge_circulator current_edge =
                shape.incident_edges(edge_target(start_edge), start_edge->first);

            double polygon_area_x2 = 0;

            // Add vertices until the polygon is implicitly closed
            // (uses a do-while because .back() is invalid until at least one loop iteration)
            do {
                Alpha_shape_2::Edge_circulator start_current_edge = current_edge;
                // Find the first regular (boundary) incident edge which hasn't been visited, going counterclockwise
                while (shape.classify(*current_edge) != Alpha_shape_2::REGULAR || is_in(visited, current_edge)) {
                    ++current_edge;

                    // FOR DEBUG
                    if (current_edge == start_current_edge) {
                        ROS_DEBUG_STREAM("Current vertex " << edge_source(current_edge)->info().pcl_index
                                                           << " connected to:");
                        while (++current_edge != start_current_edge) {
                            std::array<std::string, 4> type_str = {"EXTERIOR", "SINGULAR", "REGULAR", "INTERIOR"};
                            ROS_DEBUG_STREAM("Vertex " << edge_target(current_edge)->info().pcl_index << ", "
                                                       << type_str[shape.classify(*current_edge)]
                                                       << (is_in(visited, current_edge) ? ", visited" : ""));
                        }

                        ROS_DEBUG_STREAM("Polygon closes at vertex " << polygon_last_index);
                        ROS_DEBUG_STREAM("So far, found " << polygon.vertices.size() << " vertices in this polgon and "
                                                          << polygons.size() << " other polygons");
                        ROS_FATAL("Went into an infinite loop while finding the next edge");
                        assert(false);
                    }
                }

                // Add this edge's start vertex (aka source)
                polygon.vertices.push_back(edge_source(current_edge)->info().pcl_index);
                add_to(visited, current_edge);

                // Calculate this segment's contribution to the polygon's area
                // Formula from http://stackoverflow.com/a/1165943/522118
                polygon_area_x2 += (edge_target(current_edge)->point().x() - edge_source(current_edge)->point().x()) *
                                   (edge_target(current_edge)->point().y() + edge_source(current_edge)->point().y());
                //                ROS_DEBUG_STREAM("Adding area of edge " << edge_source(current_edge)->info().pcl_index
                //                << " -> "
                //                                                        << edge_target(current_edge)->info().pcl_index
                //                                                        << " -- current total: " << (polygon_area_x2 /
                //                                                        2));

                // Get a circulator of all edges incident to the target vertex
                current_edge = shape.incident_edges(edge_target(current_edge), current_edge->first);
            } while (polygon.vertices.back() != polygon_last_index);
            // Need to add implicit edge to the visited list (note: DON'T use current_edge, it hasn't been filtered yet)
            visited.insert(std::minmax({polygon.vertices.front(), polygon.vertices.back()}));

            // Make sure the polygon is counterclockwise
            // polygon_area_x2 is negative if counterclockwise, positive if clockwise
            if (polygon_area_x2 > 0) {
                ROS_DEBUG_STREAM("Polygon area: " << polygon_area_x2 / 2 << " -- reversing polygon");
                //                std::reverse(polygon.vertices.begin(), polygon.vertices.end());
            } else {
                ROS_DEBUG_STREAM("Polygon area: " << polygon_area_x2 / 2);
                // Code after this assumes area is positive
                polygon_area_x2 *= -1;
            }

            if (polygon_area_x2 > largest_polygon_area_x2) {
                largest_polygon_area_x2 = polygon_area_x2;
                largest_polygon_id = polygons.size();
            }

            polygons.push_back(polygon);
        }
    }

    // Put the largest polygon in the front
    std::iter_swap(polygons.begin(), polygons.begin() + largest_polygon_id);

    return std::make_tuple(boundary_points, polygons);
}

CT BuildSurface::get_simplified_triangulation(const std::vector<CustomPoint> &cgal_points,
                                              const std::vector<pcl::Vertices> &polygons) const {
    CT ct;
    for (auto &pcl_polygon : polygons) {
        auto i_to_pt = [&cgal_points](uint32_t i) { return cgal_points[i].first; };
        // I'm very smudge about correctly using tranform iterators
        // This overload of insert_constraint inserts a closed (if closed=true) polyline
        // IMPORTANT: The last boolean tells ct to close the polygon (it's implicitly closed)
        auto cid =
            ct.insert_constraint(boost::iterators::make_transform_iterator(pcl_polygon.vertices.begin(), i_to_pt),
                                 boost::iterators::make_transform_iterator(pcl_polygon.vertices.end(), i_to_pt), true);

        // Can't pass in info to insert_constraint for some reason, so set it after
        auto it = ct.vertices_in_constraint_begin(cid);
        for (auto &index : pcl_polygon.vertices) {
            assert(it != ct.vertices_in_constraint_end(cid));
            assert((*it)->point() == cgal_points[index].first);
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
    // Add the top (false) and bottom (true) faces
    add_mesh_face(mesh, transform, ct, false);
    add_mesh_face(mesh, transform, ct, true);

    // Add the sides
    for (auto cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
        for (auto vit = ct.vertices_in_constraint_begin(*cit); vit != ct.vertices_in_constraint_end(*cit); ++vit) {
            // Get the next vertex, stopping if it doesn't exist
            auto next_vit = std::next(vit);
            if (next_vit == ct.vertices_in_constraint_end(*cit)) {
                break;
            }

            // Find the face that contains vit and next_vit
            auto fit = ct.incident_faces(*vit);
            auto first_fit = fit;
            // Find the face that contains vit and next_vit and is inside the polygon
            while (!(fit->has_vertex(*next_vit) && fit->info().in_domain())) {
                ++fit;
                assert(fit != first_fit && "Went into an infinite loop looking for the face that contains the next "
                                           "vertex and is inside the polygon");
            }

            // Make the two new triangles
            shape_msgs::MeshTriangle tri1, tri2;
            // For the sake of formatting:
            const auto vit_i = (*vit)->info(), next_vit_i = (*next_vit)->info();

            if (fit->index(*next_vit) == fit->ccw(fit->index(*vit))) {
                // If next_vit is counterclockwise from next_vit, then the new triangles should be
                // <next_vit, vit, bottom(next_vit)> and <vit, bottom(vit), bottom(next_vit)>
                tri1.vertex_indices = {next_vit_i.pcl_index, vit_i.pcl_index, next_vit_i.pcl_index2};
                tri2.vertex_indices = {vit_i.pcl_index, vit_i.pcl_index2, next_vit_i.pcl_index2};
            } else {
                assert(fit->index(*next_vit) == fit->cw(fit->index(*vit)) &&
                       "Vertex `next_vit` was neither clockwise nor counterclockwise of vertex `vit`");
                // If next_vit is clockwise from next_vit, then the new triangles should be
                // <next_vit, bottom(next_vit), vit> and <vit, bottom(vit), bottom(next_vit)>
                tri1.vertex_indices = {next_vit_i.pcl_index, next_vit_i.pcl_index2, vit_i.pcl_index};
                tri2.vertex_indices = {vit_i.pcl_index, next_vit_i.pcl_index2, vit_i.pcl_index2};
            }

            // Finally, add them to the mesh
            mesh.triangles.push_back(tri1);
            mesh.triangles.push_back(tri2);
        }
    }

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
            for (int i = 0; i < 3; i++) {
                if (!it->vertex(i)->info().visited) { // Then it's not yet in the mesh meshage
                    if (bottom) {
                        it->vertex(i)->info().pcl_index2 = static_cast<uint32_t>(mesh.vertices.size());
                    } else {
                        it->vertex(i)->info().pcl_index = static_cast<uint32_t>(mesh.vertices.size());
                    }
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

                if (bottom) {
                    mesh_triangle.vertex_indices[i] = it->vertex(i)->info().pcl_index2;
                } else {
                    mesh_triangle.vertex_indices[i] = it->vertex(i)->info().pcl_index;
                }
            }

            if (bottom) {
                // Flip triangles when it's the bottom
                std::swap(mesh_triangle.vertex_indices[0], mesh_triangle.vertex_indices[2]);
            }
            mesh.triangles.push_back(mesh_triangle);
        }
    }
}
