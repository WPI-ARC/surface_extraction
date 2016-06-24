//
// Created by will on 6/22/16.
//

#include "build_surface/BuildSurface.h"

// std / Boost include

// Surface types includes
#include <surface_types/Surfaces.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>

// Utility includes
#include <surface_utils/smart_ptr.hpp>
#include <surface_utils/pcl_utils.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

BuildSurface::BuildSurface(double perpendicular_distance, double alpha)
    : perpendicular_distance_(perpendicular_distance), alpha_(alpha), next_id_(1), color_gen_(0.5, 0.99) {}

void BuildSurface::build_updated_surface(const Surface &old_surface,
                                         const std::function<void(BuildSurface::Surface)> callback) {
    pcl::ScopeTime("BuildSurface::build_updated_surface");

    Surface updated_surface;

    // ID and color don't change
    updated_surface.id = old_surface.id;
    updated_surface.color = old_surface.color;

    // Inliers was already updated
    updated_surface.inliers = old_surface.inliers;
\
    // The rest needs to be computed
    updated_surface.model = find_model_for_inliers(updated_surface.inliers, old_surface.model);
    updated_surface.pose = adjust_pose_to_model(old_surface.pose, updated_surface.model);
    auto p = ProgressListener();
    std::tie(updated_surface.boundary, updated_surface.polygons) =
        find_boundary_and_polygons(updated_surface.inliers, updated_surface.pose, p);
    //    updated_surface.polygons = simplify_polygons(updated_surface.polygons);
    //    updated_surface.mesh = create_trimesh(updated_surface.boundary, updated_surface.polygons);

    callback(updated_surface);
}

void BuildSurface::build_new_surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3d pose,
                                     ProgressListener &p, std::function<void(Surface)> callback) {
    pcl::ScopeTime("BuildSurface::build_new_surface");
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

    p.pose("surface_pose", pose);

    // boundary, polygons, and mesh are computed
    std::tie(new_surface.boundary, new_surface.polygons) =
        find_boundary_and_polygons(new_surface.inliers, new_surface.pose, p);
    p.polygons("polygons", new_surface);
    //    new_surface.polygons = simplify_polygons(new_surface.polygons);
    //    new_surface.mesh = create_trimesh(new_surface.boundary, new_surface.polygons);

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

// Utility function
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_source(T &it) {
    return it->first->vertex(it->first->cw(it->second));
};

// Utility function
template <typename T>
inline typename T::value_type::first_type::value_type::Vertex_handle edge_target(T &it) {
    return it->first->vertex(it->first->ccw(it->second));
};

std::tuple<pcl::PointIndices, std::vector<pcl::Vertices>>
BuildSurface::find_boundary_and_polygons(const PointCloud &cloud, const Eigen::Affine3d &tf, ProgressListener &p) {
    // Get a kernel (which AFAIK is in charge of geometric operations) and the relevant contained types
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::FT FT;
    typedef K::Point_2 CGALPoint;
    typedef K::Segment_2 CGALSegment;

    // Along with each point, store a PCL point index (uint32_t) and a visited boolean
    struct CustomPointInfo {
        uint32_t pcl_index;
        bool visited;

        CustomPointInfo() : pcl_index(std::numeric_limits<uint32_t>::max()), visited(false) {} // For CGAL
        CustomPointInfo(uint32_t i) : pcl_index(i), visited(false) {}                          // For me
        CustomPointInfo(uint32_t i, bool v) : pcl_index(i), visited(v) {}                      // For completeness
    };
    // Convenience typedef for the type CGAL will create for each point
    typedef std::pair<CGALPoint, CustomPointInfo> CustomPoint;
    // Get a vertex base type that stores a 2D point and my custom info
    typedef CGAL::Triangulation_vertex_base_with_info_2<CustomPointInfo, K> Vbi;
    // Get a vertex base type that works with Alpha shapes, and make it inherit from the type with my custom info
    typedef CGAL::Alpha_shape_vertex_base_2<K, Vbi> Vb;

    // Get the rest of the types required for alpha shapes
    typedef CGAL::Alpha_shape_face_base_2<K> Fb;
    typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
    typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;
    typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;

    // Make a transformed cloud
    auto cloud_transformed = boost::make_shared<PointCloud>(cloud);
    pcl::transformPointCloud(cloud, *cloud_transformed, tf.cast<float>().inverse());

    // Copy the points into a CGAL data structure and simultaneously project
    std::vector<CustomPoint> cgal_points;
    const auto proj_warn_threshold = perpendicular_distance_ * 2;
    for (std::size_t i = 0; i < cloud_transformed->size(); i++) {
        auto pt = cloud_transformed->points[i];
        if (pt.z > proj_warn_threshold || pt.z < -proj_warn_threshold) {
            ROS_WARN_STREAM("Projected point is more than " << proj_warn_threshold << "m from zero (" << pt.z << "m)");
        }
        cgal_points.emplace_back(CGALPoint(pt.x, pt.y), CustomPointInfo(i));
    }

    // Create a regularized alpha shape with the alpha set to the user's provided alpha
    Alpha_shape_2 shape(cgal_points.begin(), cgal_points.end(), alpha_, Alpha_shape_2::REGULARIZED);

    // Find the boundary indices
    pcl::PointIndices boundary_indices;
    for (auto it = shape.alpha_shape_vertices_begin(); it != shape.alpha_shape_vertices_end(); ++it) {
        if (shape.classify(*it) == Alpha_shape_2::REGULAR) {
            boundary_indices.indices.push_back((*it)->info().pcl_index);
        } else {
            assert(false && "Assumption failed -- alpha shape contains non-regular points");
        }
    }

    // Find the polygons
    std::vector<pcl::Vertices> polygons;
    for (auto start_edge = shape.alpha_shape_edges_begin(); start_edge != shape.alpha_shape_edges_end(); ++start_edge) {
        // start_edge is an iterator to a pair <Face_handle, int>, where the int is an edge of the face
        if (!edge_target(start_edge)->info().visited && shape.classify(*start_edge) == Alpha_shape_2::REGULAR) {
            pcl::Vertices polygon;

            // The start edge will be the implicitly closed edge. Start the search at its target and iterate until
            // the vector ends with its source.
            auto polygon_last_index = edge_source(start_edge)->info().pcl_index;
            auto current_edge = shape.incident_edges(edge_target(start_edge), start_edge->first);

            // Add vertices until the polygon is implicitly closed
            // (uses a do-while because .back() is invalid until at least one loop iteration)
            do {
                auto start_current_edge = current_edge;
                // Find the first regular (boundary) incident edge which hasn't been visited, going counterclockwise
                while (edge_source(current_edge)->info().visited ||
                       shape.classify(*current_edge) != Alpha_shape_2::REGULAR) {
                    ++current_edge;
                    assert(edge_target(current_edge)->info().pcl_index !=
                               edge_target(start_current_edge)->info().pcl_index &&
                           "Went into an infinite loop while finding the next edge");
                }

                // Add this edge's start vertex (aka source)
                polygon.vertices.push_back(edge_source(current_edge)->info().pcl_index);
                edge_source(current_edge)->info().visited = true;

                // Get a circulator of all edges incident to the target vertex
                current_edge = shape.incident_edges(edge_target(current_edge), current_edge->first);
            } while (polygon.vertices.back() != polygon_last_index);

            polygons.push_back(polygon);
        } else if (shape.classify(*start_edge) != Alpha_shape_2::REGULAR) {
            assert(false && "Assumption failed -- alpha shape contains non-regular edges");
        }
    }

    ROS_INFO_STREAM("Found " << boundary_indices.indices.size() << " boundary points and " << polygons.size()
                             << " polygons in the alpha shape");

    return std::make_tuple(boundary_indices, polygons);
}
