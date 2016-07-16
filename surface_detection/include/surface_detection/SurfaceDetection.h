//
// Created by will on 6/20/16.
//

#ifndef PROJECT_SURFACEDETECTION_H
#define PROJECT_SURFACEDETECTION_H

// std / Boost include
#include <future>

// SurfaceData types includes
#include <surface_types/Surfaces.hpp>

// Algorithm includes
#include <collect_points/CollectPoints.h>
#include <expand_surfaces/ExpandSurfaces.h>
#include <detect_surfaces/DetectSurfaces.h>
#include <build_surface/BuildSurface.h>

// Utils includes
#include "surface_utils/SurfaceVisualizationController.hpp"
#include <arc_utilities/maybe.hpp>

namespace surface_detection {
class SurfaceDetection {
public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::PointXYZRGB PointRGB;
    typedef pcl::PointCloud<PointRGB> RGBCloud;

    typedef surface_types::Surfaces Surfaces;

    typedef std::pair<PointCloud, pcl::PointIndices> CloudIndexPair;

    SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist,
                     double point_inside_threshold, double mls_radius, unsigned int min_pts_in_surface,
                     double min_plane_width, double alpha, float extrusion_distance, bool optimistic,
                     std::string target_frame, std::string camera_frame);

    void add_points(const PointCloud::ConstPtr &points) { collect_points_.add_points(points); }

    PointCloud get_pending_points() { return collect_points_.get_pending_points(); }
    RGBCloud get_surface_points();

    Surfaces detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents,
                                    const Surface::ProvideLevel provide_inliers,
                                    const Surface::ProvideLevel provide_shape, const Surface::ProvideLevel provide_mesh,
                                    const SurfaceVisualizationController &v);

    void add_start_surface(float start_surface_extent_x, float start_surface_extent_y,
                           const SurfaceVisualizationController &p);

    size_t get_num_pending_points();

private:
    // Configuration
    std::string target_frame_;
    double discretization_;
    double parallel_distance_;
    double perpendicular_distance_;
    double min_width_;
    double sqr_perpendicular_distance_;
    unsigned int min_pts_in_surface_;
    bool optimistic_;

    // State
    std::map<int, Surface> surfaces_;

    // Implementation
    CollectPoints collect_points_;
    ExpandSurfaces expand_surfaces_;
    DetectSurfaces detect_surfaces_;
    BuildSurface build_surface_;

    std::future<void> cleanup_done_;
    std::set<uint32_t> expansion_skipped_;

    void remove_surface(const Surface &surface);

    // This is the delegated-to one that does the work
    void find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                    const std::vector<Surface> &surfaces, const Maybe::Maybe<uint32_t> &ignore_surface,
                    const std::function<void(Maybe::Maybe<Surface>)> &callback);
    // This is the one used when calling with a new surface
    void find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                    const std::vector<Surface> &surfaces, const std::function<void(Maybe::Maybe<Surface>)> &callback);
    // This is the one used when calling with an existing surface
    void find_merge(const Surface &test_surf, const std::vector<Surface> &surfaces,
                    const std::function<void(Maybe::Maybe<Surface>)> &callback);

    const Surface &get_surface(uint32_t surface_id) const;
    Surface &get_surface(uint32_t surface_id);

    void wait_for_cleanup() const;

    void publish_deleted_surfaces(const std::set<uint32_t> &ids, const SurfaceVisualizationController &v) const;
    void publish_updated_surfaces(const std::set<uint32_t> &ids, const SurfaceVisualizationController &v) const;

    void update_collector_surfaces(const SurfaceVisualizationController &v);

    void cleanup(PointCloud &processed_pts, std::set<uint32_t> &updated, std::set<uint32_t> &deleted,
                     std::vector<int> &rm_indices, std::vector<int> labels,
                     const SurfaceVisualizationController &v);

    void add_surface(Surface surface);
};
}
#endif // PROJECT_SURFACEDETECTION_H
