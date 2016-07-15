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
#include <build_surface/cgal_types.hpp>
#include <unordered_map>

namespace pcl {
struct PointXYZ;
class PointIndices;
class Vertices;
}

class Surface;

namespace random_colors {
class color_generator;
}

class BuildSurface {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::PointXYZL LabeledPoint;
    typedef pcl::PointCloud<LabeledPoint> LabeledCloud;

    typedef std::vector<pcl::Vertices> Polygons;
    typedef std::unordered_map<uint32_t, uint32_t> Reindexer;

public:
    BuildSurface(double perpendicular_distance, double parallel_distance, double point_inside_d, double alpha,
                 float extrude_distance);

    std::tuple<pcl::ModelCoefficients, Eigen::Affine3d> compute_plane(const Surface &surface,
                                                                      const SurfaceVisualizationController &v) const;

    std::tuple<PointCloud, Polygons> compute_shape(const Surface &surface,
                                                   const SurfaceVisualizationController &v) const;

    shape_msgs::Mesh compute_mesh(const Surface &surface) const;

    std::tuple<PointCloud, Polygons, shape_msgs::Mesh>
    compute_shape_and_mesh(Surface surface, const SurfaceVisualizationController &v) const;

    std::tuple<pcl::ModelCoefficients, Eigen::Affine3d, PointCloud, Polygons, shape_msgs::Mesh>
    compute_derived(Surface surface, const SurfaceVisualizationController &p) const;

    LabeledCloud tile_surface(const Surface &surface) const;

protected:
    double perpendicular_distance_;
    double parallel_distance_;
    double point_inside_threshold_;
    double alpha_;
    float extrude_distance_;

private:
    std::tuple<PointCloud, Polygons, shape_msgs::Mesh>
    compute_shape_and_mesh(Surface surface, const Eigen::Affine3f &pose_float,
                           const SurfaceVisualizationController &v) const;

    std::tuple<PointCloud, Polygons, Reindexer>
    compute_boundary_polygons(const PointCloud &cloud, const Eigen::Affine3f &pose,
                              const std::vector<CustomPoint> &cgal_pts, const SurfaceVisualizationController &v) const;

    shape_msgs::Mesh compute_mesh(const std::vector<CustomPoint> &cgal_pts, const Polygons &polygons,
                                  const Eigen::Affine3f &pose) const;

    pcl::ModelCoefficients optimize_model_for_inliers(const PointCloud &inliers,
                                                      const pcl::ModelCoefficients &old_coeff) const;

    pcl::ModelCoefficients find_model_for_inliers(const PointCloud &cloud,
                                                  const pcl::ModelCoefficients &prev_model) const;

    Eigen::Affine3d adjust_pose_to_model(Eigen::Affine3d pose, pcl::ModelCoefficients model,
                                         const SurfaceVisualizationController &p) const;

    std::tuple<BuildSurface::PointCloud, Polygons, Reindexer>
    find_boundary_and_polygons(const PointCloud &cloud, const std::vector<CustomPoint> &cgal_points,
                               const Eigen::Affine3f &transform, const SurfaceVisualizationController &p) const;

    Polygons simplify_polygons(const CT &ct) const;

    std::vector<CustomPoint> get_cgal_2d_points(const PointCloud &cloud, const Eigen::Affine3f &transform) const;

    CT get_simplified_triangulation(const std::vector<CustomPoint> &cgal_points, const Polygons &polygons) const;

    shape_msgs::Mesh create_trimesh(CT ct, const Eigen::Affine3f &transform) const;

    void add_mesh_face(shape_msgs::Mesh &mesh, const Eigen::Affine3f &transform, const CT &ct, bool bottom) const;

    std::tuple<PointCloud, Reindexer> get_boundary_from_alpha(const Alpha_shape_2 &shape,
                                                              const Eigen::Affine3f &transform) const;

    void reindex_polygons(Polygons &polygons, const Reindexer &reindexer) const;
};

#endif // PROJECT_BuildSurfaces_H
