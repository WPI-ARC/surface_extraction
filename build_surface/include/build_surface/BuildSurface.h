//
// Created by will on 6/22/16.
//

#ifndef PROJECT_BuildSurfaces_H
#define PROJECT_BuildSurfaces_H

#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <surface_utils/color_generator.hpp>
#include <surface_utils/ProgressListener.hpp>

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

class BuildSurface {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;


    typedef surface_types::Surface Surface;

public:
    BuildSurface(double perpendicular_distance, double alpha);

    void build_updated_surface(const Surface &old_surface, const std::function<void(Surface)> callback);

    void build_new_surface(PointCloud inliers, pcl::ModelCoefficients model, Eigen::Affine3d pose, ProgressListener &p,
                               std::function<void(Surface)> callback);

protected:
    double perpendicular_distance_;
    double alpha_;

    uint32_t next_id_;
    random_colors::color_generator color_gen_;

private:
    pcl::ModelCoefficients find_model_for_inliers(const PointCloud &inliers);
    pcl::ModelCoefficients find_model_for_inliers(const PointCloud &cloud, const pcl::ModelCoefficients &prev_model);

    Eigen::Affine3d adjust_pose_to_model(Eigen::Affine3d old_pose, pcl::ModelCoefficients model);

    std::tuple<pcl::PointIndices, std::vector<pcl::Vertices>> find_boundary_and_polygons(
            const PointCloud &cloud, const Eigen::Affine3d &model, ProgressListener &p);
};

#endif // PROJECT_BuildSurfaces_H
