//
// Created by will on 6/20/16.
//

#ifndef PROJECT_SURFACEDETECTION_H
#define PROJECT_SURFACEDETECTION_H

// std / Boost include
#include <future>

// Surface types includes
#include <surface_types/Surfaces.hpp>

// Algorithm includes
#include <collect_points/CollectPoints.h>
#include <expand_surfaces/ExpandSurfaces.h>
#include <detect_surfaces/DetectSurfaces.h>
#include <build_surface/BuildSurface.h>
#include <surface_utils/pcl_utils.hpp>

// Utils includes
#include "surface_utils/ProgressListener.hpp"
#include <pcl/common/time.h>

namespace surface_detection {
class SurfaceDetection {
public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef surface_types::Surface Surface;
    typedef surface_types::Surfaces Surfaces;

    SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist, double mls_radius,
                     unsigned int min_pts_in_surface, double min_plane_width, double alpha)
        : // State
          surfaces_(),
          // Implementation
          collect_points_(discretization, perpendicular_dist), expand_surfaces_(perpendicular_dist, parallel_dist),
          detect_surfaces_(perpendicular_dist, parallel_dist, mls_radius, min_pts_in_surface, min_plane_width),
          build_surface_(perpendicular_dist, alpha) {}

    void add_points(const PointCloud::ConstPtr &points) { collect_points_.add_points(points); }

    PointCloud get_pending_points() { return collect_points_.get_pending_points(); }

    Surfaces detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents, ProgressListener p) {
        pcl::ScopeTime("SurfaceDetection::detect_surfaces_within");
        // Get points to process
        auto points_to_process = collect_points_.pending_points_within(center, extents);
        ROS_DEBUG_STREAM("Processing " << points_to_process.second.indices.size() << " points");
        p.pair("points_to_process", points_to_process);

        std::vector<Surface> surfaces;

        // Expand surfaces
        points_to_process.second = expand_surfaces_.expand_surfaces(surfaces_, points_to_process, [&, this](Surface s) {
            build_surface_.build_updated_surface(s, [&, this](Surface updated_surface) {
                surfaces.push_back(updated_surface);
                ROS_DEBUG_STREAM("Got an expanded version of surface " << s.id);
            });
        });
        ROS_DEBUG_STREAM("Expanded surfaces, " << points_to_process.second.indices.size() << " points remaining");

        // Detect new surfaces
        detect_surfaces_.detect_surfaces(
            points_to_process, p, [&](pcl::PointIndices indices, pcl::ModelCoefficients model, Eigen::Affine3f tf) {
                    auto inliers_cloud = boost::make_shared<PointCloud>(points_to_process.first, indices.indices);

                    build_surface_.build_new_surface(*inliers_cloud, model,
                                                     tf.cast<double>(), p, [&, this](Surface new_surface) {
                                surfaces.push_back(new_surface);
                                ROS_DEBUG_STREAM("Got a new surface");
                            });
            });
        ROS_DEBUG_STREAM("Finished detecting surfaces");

        return {};
    }

    Surfaces detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        return detect_surfaces_within(center, extents, ProgressListener());
    }

private:
    // State
    std::map<int, Surface> surfaces_;

    // Implementation
    CollectPoints collect_points_;
    ExpandSurfaces expand_surfaces_;
    DetectSurfaces detect_surfaces_;
    BuildSurface build_surface_;
};
}
#endif // PROJECT_SURFACEDETECTION_H
