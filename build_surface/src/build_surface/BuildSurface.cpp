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
#include <arc_utilities/pretty_print.hpp>

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <surface_utils/geom_utils.hpp>
#include <pcl/search/kdtree.h>
#include <build_surface/cgal_utils.hpp>

using surfaces_pcl_utils::model_normal;

BuildSurface::BuildSurface(double perpendicular_distance, double parallel_distance, double point_inside_d, double alpha,
                           float extrude_distance)
    : perpendicular_distance_(perpendicular_distance), parallel_distance_(parallel_distance),
      point_inside_threshold_(point_inside_d), alpha_(alpha), extrude_distance_(extrude_distance) {}

std::tuple<pcl::ModelCoefficients, Eigen::Affine3d>
BuildSurface::compute_plane(const Surface &surface, const SurfaceVisualizationController &v) const {
    auto model = find_model_for_inliers(surface.inliers(), surface.model_approx());
    auto pose = adjust_pose_to_model(surface.pose_approx(), model, v);

    return std::make_tuple(model, pose);
}

std::tuple<BuildSurface::PointCloud, BuildSurface::Polygons>
BuildSurface::compute_shape(const Surface &surface, const SurfaceVisualizationController &v) const {
    // Always call get_cgal_2d_points with whichever cloud the polygons refer to. These polygons will refer to inliers.
    auto cgal_points = get_cgal_2d_points(surface.inliers(), surface.pose_float());

    PointCloud boundary;
    Polygons polygons;
    Reindexer reindexer;

    std::tie(boundary, polygons, reindexer) = compute_boundary_polygons(surface.inliers(), surface.pose_float(), cgal_points, v);
    reindex_polygons(polygons, reindexer);

    assert(std::all_of(polygons.begin(), polygons.end(), [&boundary](const pcl::Vertices &poly) {
        return std::all_of(poly.vertices.begin(), poly.vertices.end(),
                           boost::bind(std::less<uint32_t>(), _1, boundary.size()));
    }) && "Reindexer gave an index outside of the boundary cloud");

    return std::make_pair(boundary, polygons);
}

shape_msgs::Mesh BuildSurface::compute_mesh(const Surface &surface) const {
    // Always call get_cgal_2d_points with whichever cloud the polygons refer to. These polygons refer to boundary.
    auto cgal_points = get_cgal_2d_points(surface.boundary(), surface.pose_float());
    auto ct = get_simplified_triangulation(cgal_points, surface.polygons());
    return create_trimesh(ct, surface.pose_float());
}

std::tuple<pcl::ModelCoefficients, Eigen::Affine3d, BuildSurface::PointCloud, BuildSurface::Polygons, shape_msgs::Mesh>
BuildSurface::compute_derived(Surface surface, const SurfaceVisualizationController &v) const {
    auto plane = compute_plane(surface, v);
    auto shape_and_mesh = compute_shape_and_mesh(surface, std::get<1>(plane).cast<float>(), v);

    return std::tuple_cat(plane, shape_and_mesh);
}

std::tuple<BuildSurface::PointCloud, BuildSurface::Polygons, shape_msgs::Mesh>
BuildSurface::compute_shape_and_mesh(Surface surface, const SurfaceVisualizationController &v) const {
    return compute_shape_and_mesh(surface, surface.pose_float(), v);
};

std::tuple<BuildSurface::PointCloud, BuildSurface::Polygons, shape_msgs::Mesh>
BuildSurface::compute_shape_and_mesh(Surface surface, const Eigen::Affine3f &pose_float,
                                     const SurfaceVisualizationController &v) const {
    // Always call get_cgal_2d_points with whichever cloud the polygons refer to. These polygons will refer to inliers.
    auto cgal_points = get_cgal_2d_points(surface.inliers(), pose_float);
    PointCloud boundary;
    Polygons polygons;
    Reindexer reindexer;
    std::tie(boundary, polygons, reindexer) = compute_boundary_polygons(surface.inliers(), pose_float, cgal_points, v);

    auto mesh = compute_mesh(cgal_points, polygons, pose_float);

    reindex_polygons(polygons, reindexer);
    assert(std::all_of(polygons.begin(), polygons.end(), [&boundary](const pcl::Vertices &poly) {
        return std::all_of(poly.vertices.begin(), poly.vertices.end(),
                           boost::bind(std::less<uint32_t>(), _1, boundary.size()));
    }) && "Reindexer gave an index outside of the boundary cloud");

    return std::make_tuple(boundary, polygons, mesh);
};

std::tuple<BuildSurface::PointCloud, BuildSurface::Polygons, BuildSurface::Reindexer>
BuildSurface::compute_boundary_polygons(const PointCloud &cloud, const Eigen::Affine3f &pose,
                                        const std::vector<CustomPoint> &cgal_points,
                                        const SurfaceVisualizationController &v) const {
    auto tup = find_boundary_and_polygons(cloud, cgal_points, pose, v);

    auto triangulation = get_simplified_triangulation(cgal_points, std::get<1>(tup));
    std::get<1>(tup) = simplify_polygons(triangulation);

    return tup;
}

shape_msgs::Mesh BuildSurface::compute_mesh(const std::vector<CustomPoint> &cgal_points, const Polygons &polygons,
                                            const Eigen::Affine3f &pose) const {
    auto ct = get_simplified_triangulation(cgal_points, polygons);
    return create_trimesh(ct, pose);
}

pcl::ModelCoefficients BuildSurface::optimize_model_for_inliers(const PointCloud &inliers,
                                                                const pcl::ModelCoefficients &model) const {
    auto weighted_inliers = boost::make_shared<PointCloud>();
    std::vector<int> weighted_indices;
//! TODO Add a config parameter for whether to use weighted PCA (currently disabled because it might be broken)
#ifdef USE_WEIGHTED_PCA
    // Weight each point by its number of neighbors. This is a proxy for weighting it by its distance to the edge.
    // TODO: Figure out when it's faster to use a pcl::search::BruteForce
    pcl::search::KdTree<Point> search(false);
    search.setInputCloud(boost::shared_ptr<const PointCloud>(&inliers, null_deleter()));
    Eigen::ArrayXf weights(inliers.size());
    Eigen::RowVector4f mean = Eigen::RowVector4f::Zero();
    const auto radius = 2 * parallel_distance_;
    for (std::size_t i = 0; i < inliers.size(); i++) {
        std::vector<int> neighbors;
        std::vector<float> neighbor_sqr_dist;
        search.radiusSearch(static_cast<int>(i), radius, neighbors, neighbor_sqr_dist);
        weights[i] = std::accumulate(neighbor_sqr_dist.begin(), neighbor_sqr_dist.end(), 0.f);

        // Don't use zero-weighted points to optimize model
        if (weights[i] != 0) {
            weighted_indices.push_back(static_cast<int>(i));
        } else {
            ROS_DEBUG_STREAM("Index " << i << " weight was zero");
        }

        ROS_ERROR_STREAM_COND(std::isnan(weights[i]), "Got a NaN weight " << i
                                                                          << ", n_neighbors: " << neighbors.size());

        mean += inliers[i].getVector4fMap();

        ROS_ERROR_STREAM_COND(mean.hasNaN(), "Mean was NaN at " << i << ", pt "
                                                                << inliers[i].getVector4fMap().transpose());
    }
    mean /= inliers.size();
    weights /= weights.maxCoeff();

    // Build the weighted vector
    weighted_inliers->resize(inliers.size()); // Important, or the next line will be invalid
    ROS_DEBUG_STREAM("Inliers size " << inliers.size() << " map size " << inliers.getMatrixXfMap().cols()
                                     << " weights size " << weights.rows());
    assert(inliers.getMatrixXfMap().cols() == weights.rows() && "Weighting vector constructed wrong");
    // This line is all Eigen arrays (coefficient-wise ops) except the (1-w) * mean, which is the outer vector
    // product
    const auto weighted_pts = inliers.getMatrixXfMap().array().rowwise() * weights.transpose();
    ROS_DEBUG_STREAM("weights: " << weights.rows() << "x" << weights.cols());
    const auto subweights = 1 - weights;
    ROS_DEBUG_STREAM("subweights: " << subweights.rows() << "x" << subweights.cols());
    const auto subweightsm = subweights.matrix();
    ROS_DEBUG_STREAM("subweightsm: " << subweightsm.rows() << "x" << subweightsm.cols());
    const auto weightmean = subweightsm * mean;
    ROS_DEBUG_STREAM("weightmean: " << weightmean.rows() << "x" << weightmean.cols());
    const auto weightmeana = weightmean.array();
    ROS_DEBUG_STREAM("weightmeana: " << weightmeana.rows() << "x" << weightmeana.cols());
    const auto weighted_means = weightmeana.transpose();
    ROS_DEBUG_STREAM("weighted_means: " << weighted_means.rows() << "x" << weighted_means.cols());
    ROS_DEBUG_STREAM("Pts is " << weighted_pts.rows() << "x" << weighted_pts.cols() << ", means is "
                               << weighted_means.rows() << "x" << weighted_means.cols());
    weighted_inliers->getMatrixXfMap() = weighted_pts + weighted_means;

    assert(!weighted_inliers->getMatrixXfMap().hasNaN() && "Some of the inliers were NaN :/");
#else
    weighted_inliers = boost::make_shared<PointCloud>(inliers); // Makes a copy
    weighted_indices = surfaces_pcl_utils::all_indices(inliers);
#endif
    // Note that operations such as countWithinDistance on this sacmodel operate on the weighted cloud
    pcl::SampleConsensusModelPlane<Point> sacmodel(weighted_inliers);

    Eigen::VectorXf coefficients(4), old_co(4);
    old_co << model.values[0], model.values[1], model.values[2], model.values[3];

    assert(!old_co.hasNaN() && "Previous coefficients had NaN");

    sacmodel.optimizeModelCoefficients(weighted_indices, old_co, coefficients);

    assert(!coefficients.hasNaN() && "Optimized coefficients had NaN");

    if (false) {
        auto points_outside = inliers.size() - sacmodel.countWithinDistance(coefficients, perpendicular_distance_);

        if (points_outside > 0.05 * inliers.size()) {
            ROS_ERROR_STREAM("New model excludes " << static_cast<double>(points_outside) / inliers.size() * 100
                                                   << "% of points (" << points_outside << " / " << inliers.size()
                                                   << ")");
            // TODO Incorporate the idea that a surface construction can fail, and make it fail here
            // (probably using exceptions)
        } else if (points_outside > 0.01 * inliers.size()) {
            ROS_WARN_STREAM("New model excludes " << static_cast<double>(points_outside) / inliers.size() * 100
                                                  << "% of points (" << points_outside << " / " << inliers.size()
                                                  << ")");
        }
    }

    pcl::ModelCoefficients ret;
    ret.header = inliers.header;
    ret.values = {coefficients[0], coefficients[1], coefficients[2], coefficients[3]};

    return ret;
}

pcl::ModelCoefficients BuildSurface::find_model_for_inliers(const PointCloud &cloud,
                                                            const pcl::ModelCoefficients &prev_model) const {
    auto new_model = optimize_model_for_inliers(cloud, prev_model);

    assert(new_model.values.size() == 4 && "optimize_model_for_inliers gave an invalid model");
    assert(prev_model.values.size() == 4 && "find_model_for_inliers recieved invalid prev_model");

    float d = model_normal(new_model).dot(model_normal(prev_model));

    // If dot product is less than 0, the vectors (which represent normal vectors) were more than 90deg apart
    if (d < 0) {
        ROS_DEBUG_STREAM("Flipping optimized plane normal");
        new_model.values[0] *= -1;
        new_model.values[1] *= -1;
        new_model.values[2] *= -1;
        new_model.values[3] *= -1;
    }

    return new_model;
}

Eigen::Affine3d BuildSurface::adjust_pose_to_model(Eigen::Affine3d pose, pcl::ModelCoefficients model,
                                                   const SurfaceVisualizationController &p) const {
    // Promote all the floats coming from model to doubles
    const auto plane_normal = Eigen::Map<Eigen::Vector3f>(model.values.data()).cast<double>();
    const auto plane_distance = static_cast<double>(model.values[3]);
    assert(std::abs(plane_normal.norm() - 1) < 1e-2 && "Plane normal from model was not a unit vector");

    Eigen::Affine3d pose_before_inv(pose.inverse());
    p.pose("pose_before", pose_before_inv * pose); // Yes, this is intentionally identity

    // 1. Rotate to align z^ with the plane normal
    // (Rotate the pose by the same quaterion that aligns z^ with the plane normal, expressed relative to the pose)
    const auto normal_relative_to_pose = pose.rotation().inverse() * plane_normal;
    p.vector("normal", normal_relative_to_pose, Eigen::Vector3d::Zero());
    auto quaternion = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal_relative_to_pose);
    pose.rotate(quaternion);

    geometry_msgs::Pose geom_pose;
    tf::quaternionEigenToMsg(quaternion, geom_pose.orientation);
    p.pose("rotation", geom_pose);

    p.pose("pose_after", pose_before_inv * pose);

    // At this point, z axis should be identical to the plane normal (within floating point error)
    ROS_ERROR_STREAM_COND(!(pose.linear() * Eigen::Vector3d::UnitZ()).isApprox(plane_normal, 1e-06),
                          "Pose z-axis is not identical to normal -- difference is "
                              << ((pose.linear() * Eigen::Vector3d::UnitZ()) - plane_normal).transpose());

    assert((pose.linear() * Eigen::Vector3d::UnitZ()).isApprox(plane_normal, 1e-06) &&
           "Didn't correctly orient the transform's z axis to the plane normal");

    // 3. Translate by in the z^ direction by the negative signed point-plane distance to put pose's origin on the plane
    pose.translate(Eigen::Vector3d::UnitZ() * -(pose.translation().dot(plane_normal) + plane_distance));

    p.pose("pose_after_adjust_translation", pose);

    // At this point, the tf's origin should be on the plane
    assert(std::abs(pose.translation().dot(plane_normal) + plane_distance) < 1e-5 &&
           "Didn't correctly translate the pose to put origin on the plane");

    return pose;
}

std::vector<CustomPoint> BuildSurface::get_cgal_2d_points(const PointCloud &cloud,
                                                          const Eigen::Affine3f &transform) const {
    // Make a transformed cloud
    auto cloud_transformed = boost::make_shared<PointCloud>(cloud);
    pcl::transformPointCloud(cloud, *cloud_transformed, transform.inverse());

    // Copy the points into a CGAL data structure and simultaneously project
    std::vector<CustomPoint> cgal_points;
    const auto proj_warn_threshold = perpendicular_distance_ * 2;
    auto proj_warn_n = 5;
    for (std::size_t i = 0; i < cloud_transformed->size(); i++) {
        auto pt = cloud_transformed->points[i];
        if (pt.z > proj_warn_threshold || pt.z < -proj_warn_threshold) {
            if (proj_warn_n > 0) {
                ROS_WARN_STREAM("Projected point is more than " << proj_warn_threshold << "m from zero (" << pt.z
                                                                << "m)");
            }
            proj_warn_n--;
        }
        cgal_points.emplace_back(CGALPoint(pt.x, pt.y), CustomPointInfo(static_cast<uint32_t>(i)));
    }

    if (proj_warn_n < 0) {
        ROS_WARN_STREAM("An additional " << -proj_warn_n << " points were more than " << proj_warn_threshold
                                         << "m from zero");
    }

    return cgal_points;
}

std::tuple<BuildSurface::PointCloud, BuildSurface::Polygons, BuildSurface::Reindexer>
BuildSurface::find_boundary_and_polygons(const PointCloud &cloud, const std::vector<CustomPoint> &cgal_points,
                                         const Eigen::Affine3f &transform,
                                         const SurfaceVisualizationController &p) const {

    // Create a regularized alpha shape with the alpha set to the user's provided alpha
    Alpha_shape_2 shape(cgal_points.begin(), cgal_points.end(), alpha_, Alpha_shape_2::REGULARIZED);

    // For debug -- check that every edge has an even number of regular
    for (auto start_edge = shape.alpha_shape_edges_begin(); start_edge != shape.alpha_shape_edges_end(); ++start_edge) {
    }
    // Find the boundary indices
    BuildSurface::PointCloud boundary_points;
    Reindexer reindexer;
    std::tie(boundary_points, reindexer) = get_boundary_from_alpha(shape, transform);

    std::set<std::pair<uint32_t, uint32_t>> visited;

    // Find the polygons
    Polygons polygons;
    std::size_t largest_polygon_id = 0;
    double largest_polygon_area_x2 = 0;
    for (auto start_edge = shape.alpha_shape_edges_begin(); start_edge != shape.alpha_shape_edges_end(); ++start_edge) {
        assert(shape.classify(*start_edge) == Alpha_shape_2::REGULAR && "Polygon gave a non-regular edge");
        assert(!shape.is_infinite(*start_edge) && "Alpha shape returned an infinite edge");
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
            Alpha_shape_2::Edge_circulator latest_added_edge; //! For debugging
            do {
                Alpha_shape_2::Edge_circulator start_current_edge = current_edge;
                // Find the first regular (boundary) incident edge which hasn't been visited, going counterclockwise
                while (shape.classify(*current_edge) != Alpha_shape_2::REGULAR || is_in(visited, current_edge)) {
                    ++current_edge;
                    assert(current_edge != start_current_edge &&
                           "Infinite loop while finding the next edge in an alpha shape");
                }

                // Add this edge's start vertex (aka source)
                polygon.vertices.push_back(edge_source(current_edge)->info().pcl_index);
                // Important conditional! Without this, in the (rare) case there are two polygons that share a
                // point and the start edge for the first polygon happens to begin at the shared point,
                // current_edge will point to an edge in the other polygon that shares the point, and marking it as
                // visited will cause the other polgon's construction to fail because it will reach the
                // incorrectly-visited edge and not be able to proceed because it is visited.
                // This took me literal weeks to debug.
                if (polygon.vertices.back() != polygon_last_index) {
                    add_to(visited, current_edge);
                }
                latest_added_edge = current_edge;

                // Calculate this segment's contribution to the polygon's area
                // Formula from http://stackoverflow.com/a/1165943/522118
                polygon_area_x2 += (edge_target(current_edge)->point().x() - edge_source(current_edge)->point().x()) *
                                   (edge_target(current_edge)->point().y() + edge_source(current_edge)->point().y());

                // Get a circulator of all edges incident to the target vertex
                current_edge = shape.incident_edges(edge_target(current_edge), current_edge->first);
            } while (polygon.vertices.back() != polygon_last_index);
            // Need to add implicit edge to the visited list (note: DON'T use current_edge, it hasn't been filtered yet)
            visited.insert(std::minmax({polygon.vertices.front(), polygon.vertices.back()}));

            // Make sure the polygon is counterclockwise
            // polygon_area_x2 is negative if counterclockwise, positive if clockwise
            if (polygon_area_x2 > 0) {
                std::reverse(polygon.vertices.begin(), polygon.vertices.end());
            } else {
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
    if (largest_polygon_id != 0) {
        std::iter_swap(polygons.begin(), polygons.begin() + largest_polygon_id);
    }

    return std::make_tuple(boundary_points, polygons, reindexer);
}

std::tuple<BuildSurface::PointCloud, BuildSurface::Reindexer>
BuildSurface::get_boundary_from_alpha(const Alpha_shape_2 &shape, const Eigen::Affine3f &transform) const {
    PointCloud boundary_points;
    Reindexer reindex_map;
    assert(shape.get_mode() == Alpha_shape_2::REGULARIZED &&
           "get_boundary_from_alpha assumes shape will be regularized");
    for (auto it = shape.alpha_shape_vertices_begin(); it != shape.alpha_shape_vertices_end(); ++it) {
        Point pt((*it)->point().x(), (*it)->point().y(), 0);
        reindex_map[(*it)->info().pcl_index] = boundary_points.size();
        boundary_points.push_back(transformPoint(pt, transform));
    }

    return std::make_tuple(boundary_points, reindex_map);
}

CT BuildSurface::get_simplified_triangulation(const std::vector<CustomPoint> &cgal_points,
                                              const Polygons &polygons) const {
    CT ct;
    for (auto &pcl_polygon : polygons) {
        auto i_to_pt = [&cgal_points](uint32_t i) { return cgal_points[i].first; };
        // I'm very smudge about correctly using tranform iterators
        // This overload of insert_constraint inserts a closed (if closed=true) polyline
        auto close = pcl_polygon.vertices.front() != pcl_polygon.vertices.back();
        auto cid =
            ct.insert_constraint(boost::iterators::make_transform_iterator(pcl_polygon.vertices.begin(), i_to_pt),
                                 boost::iterators::make_transform_iterator(pcl_polygon.vertices.end(), i_to_pt), close);

        // Can't pass in info to insert_constraint for some reason, so set it after
        auto it = ct.vertices_in_constraint_begin(cid);
        for (auto &index : pcl_polygon.vertices) {
            assert(it != ct.vertices_in_constraint_end(cid) && "Polygon was larger than the constraint");
            assert((*it)->point() == cgal_points[index].first && "Polygon indices out of sync with cgal points vector");
            (*it)->info() = cgal_points[index].second;
            ++it;
        }
        if (close) {
            assert((*it)->point() == cgal_points[pcl_polygon.vertices.front()].first &&
                   "Constraint was not closed correctly (last Point does not equal first pPoint)");
            ++it;
        }
        assert(it == ct.vertices_in_constraint_end(cid) && "Polygon was shorter than the constraint");
    }

    // TODO should alpha be re-used for this?
    simplify(ct, SimplificationCost(), SimplificationStop(alpha_ * alpha_));
    return ct;
}

BuildSurface::Polygons BuildSurface::simplify_polygons(const CT &ct) const {
    Polygons simplified;
    for (auto cit = ct.constraints_begin(); cit != ct.constraints_end(); ++cit) {
        pcl::Vertices polygon;
        for (auto vit = ct.vertices_in_constraint_begin(*cit); vit != ct.vertices_in_constraint_end(*cit); ++vit) {
            polygon.vertices.push_back((*vit)->info().pcl_index);
        }
        simplified.push_back(polygon);
    }

    return simplified;
}

shape_msgs::Mesh BuildSurface::create_trimesh(CT ct, const Eigen::Affine3f &transform) const {
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

BuildSurface::LabeledCloud BuildSurface::tile_surface(const Surface &surface) const {
    LabeledCloud cloud, cloud_out;
    PointCloud boundary_flat;
    pcl::transformPointCloud(surface.boundary(), boundary_flat, surface.pose().inverse());

    assert(std::all_of(boundary_flat.begin(), boundary_flat.end(), [](Point p) { return p.z < .5 && p.z > -.5; }));

    Point min, max;
    pcl::getMinMax3D(boundary_flat, min, max);

    double discretization = perpendicular_distance_ / 2.; // arbitrary
    // Offset by discretization / 2 to avoid testing the minimum, which will by definition be outside
    for (float y = static_cast<float>(min.y + discretization / 2.); y < max.y; y += discretization) {
        // X is the widest axis so iterate x most rapidly (not sure if that makes any difference w/ this algorithm)
        for (float x = static_cast<float>(min.x + discretization / 2.); x < max.x; x += discretization) {
            // For each (x, y) pair, skip if it's outside, or if it's within perpendicular_distance_ plus the maximum
            // possible discretization error of any edge
            if (surface_geom_utils::is_xy_in_tiling(surface, boundary_flat, x, y,
                                                    point_inside_threshold_ + discretization)) {
                LabeledPoint pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0;
                pt.label = surface.id();
                cloud.push_back(pt);
            }
        }
    }

    pcl::transformPointCloud(cloud, cloud_out, surface.pose_float());
    return cloud_out;
}

void BuildSurface::reindex_polygons(Polygons &polygons, const Reindexer &reindexer) const {
    // Reindex the polygon to point to boundary instead of cloud
    for (auto &polygon : polygons) {
        for (std::size_t i = 0; i < polygon.vertices.size(); i++) {
            polygon.vertices[i] = reindexer.at(polygon.vertices[i]);
        }
    }
}
