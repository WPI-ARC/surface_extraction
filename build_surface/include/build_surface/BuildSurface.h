//
// Created by will on 6/22/16.
//

#ifndef PROJECT_BuildSurfaces_H
#define PROJECT_BuildSurfaces_H

#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <surface_utils/color_generator.hpp>
#include <surface_utils/SurfaceVisualizationController.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_triangulation_plus_2.h>

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>

namespace pcl {
struct PointXYZ;
class PointIndices;
class Vertices;
}

namespace surface_types {
class Surface;
class SurfaceMesh;
}

namespace random_colors {
class color_generator;
}

//////////////////////////////////////// HERE THERE BE CGAL ////////////////////////////////////////

// Along with each point, store a PCL point index (uint32_t) and a visited boolean
struct CustomPointInfo {
    uint32_t pcl_index;
    uint32_t pcl_index2;
    bool visited;

    // For CGAL
    CustomPointInfo() : pcl_index(std::numeric_limits<uint32_t>::max()), pcl_index2(pcl_index), visited(false) {}
    // For me
    CustomPointInfo(uint32_t i) : pcl_index(i), pcl_index2(std::numeric_limits<uint32_t>::max()), visited(false) {}
    // For completeness
    CustomPointInfo(uint32_t i, uint32_t i2, bool v) : pcl_index(i), pcl_index2(i), visited(v) {}
};

struct CustomFaceInfo {
    CustomFaceInfo() : nesting_level(-1) {}
    int nesting_level;
    bool in_domain() { return nesting_level % 2 == 1; }
};

// Alias the fairly long polyline simplification namespace
namespace PS = CGAL::Polyline_simplification_2;

// Get a kernel (which AFAIK is in charge of geometric operations) and the relevant contained types
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_2 CGALPoint;
typedef K::Segment_2 CGALSegment;
// Convenience typedef for the type CGAL will create for each point
typedef std::pair<CGALPoint, CustomPointInfo> CustomPoint;
// Get a vertex base type that stores a 2D point and my custom info
typedef CGAL::Triangulation_vertex_base_with_info_2<CustomPointInfo, K> CustomVb;
// Get a vertex base type that works with Alpha shapes, and make it inherit from the type with my custom info
typedef CGAL::Alpha_shape_vertex_base_2<K, CustomVb> AlphaVb;

// Get the rest of the types required for alpha shapes
typedef CGAL::Alpha_shape_face_base_2<K> AlphaFb;
typedef CGAL::Triangulation_data_structure_2<AlphaVb, AlphaFb> AlphaTds;
typedef CGAL::Delaunay_triangulation_2<K, AlphaTds> Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;

// Get the rest of the types required for simplification
typedef PS::Vertex_base_2<K> SimplificationVbb;
typedef CGAL::Triangulation_vertex_base_with_info_2<CustomPointInfo, K, SimplificationVbb> SimplificationVb;
typedef CGAL::Constrained_triangulation_face_base_2<K> SimplificationFbb;
typedef CGAL::Triangulation_face_base_with_info_2<CustomFaceInfo, K, SimplificationFbb> SimplificationFb;
typedef CGAL::Triangulation_data_structure_2<SimplificationVb, SimplificationFb> SimplificationTds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, SimplificationTds, CGAL::Exact_predicates_tag> CDT;
typedef CGAL::Constrained_triangulation_plus_2<CDT> CT;
typedef PS::Stop_above_cost_threshold SimplificationStop;
typedef PS::Squared_distance_cost SimplificationCost;
//////////////////////////////////////// END CGAL ////////////////////////////////////////

class BuildSurface {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef surface_types::Surface Surface;

public:
    BuildSurface(double perpendicular_distance, double parallel_distance, double point_inside_d, double alpha, float extrude_distance);

    void build_updated_surface(const Surface &old_surface, const SurfaceVisualizationController &p,
                               const std::function<void(BuildSurface::Surface)> callback);

    void build_new_surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3f pose,
                           const SurfaceVisualizationController &p,
                           std::function<void(Surface)> callback);

    BuildSurface::PointCloud find_surface_boundary(const BuildSurface::PointCloud &inliers,
                                                   const Eigen::Affine3f &transform);

    Surface new_partial_surface(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                                const Eigen::Affine3f &transform);

    PointCloud tile_surface(const Surface &surface);

protected:
    double perpendicular_distance_;
    double parallel_distance_;
    double point_inside_threshold_;
    double alpha_;
    float extrude_distance_;

    uint32_t next_id_;
    random_colors::color_generator color_gen_;

private:
    pcl::ModelCoefficients optimize_model_for_inliers(const PointCloud &inliers,
                                                      const pcl::ModelCoefficients &old_coeff);
    pcl::ModelCoefficients find_model_for_inliers(const PointCloud &cloud, const pcl::ModelCoefficients &prev_model);

    Eigen::Affine3d adjust_pose_to_model(Eigen::Affine3d pose, pcl::ModelCoefficients model,
                                             const SurfaceVisualizationController &p);

    std::tuple<BuildSurface::PointCloud, std::vector<pcl::Vertices>>
    find_boundary_and_polygons(const PointCloud &cloud, const std::vector<CustomPoint> &cgal_points,
                               const Eigen::Affine3f &transform, const SurfaceVisualizationController &p);

    std::vector<pcl::Vertices> simplify_polygons(const CT &ct);

    std::vector<CustomPoint> get_cgal_2d_points(const PointCloud &cloud, const Eigen::Affine3f &transform);

    CT get_simplified_triangulation(const std::vector<CustomPoint> &cgal_points,
                                    const std::vector<pcl::Vertices> &polygons) const;

    shape_msgs::Mesh create_trimesh(CT ct, const Eigen::Affine3f &transform);

    void add_mesh_face(shape_msgs::Mesh &mesh, const Eigen::Affine3f &transform, const CT &ct, bool bottom) const;

    PointCloud get_boundary_from_alpha(const Alpha_shape_2 &shape, const Eigen::Affine3f &transform,
                                       const SurfaceVisualizationController &p) const;

    PointCloud get_boundary_from_alpha(const Alpha_shape_2 &shape, const Eigen::Affine3f &transform) const;

};

#endif // PROJECT_BuildSurfaces_H
