//
// Created by will on 6/29/16.
//

#include "surface_detection/SurfaceDetection.h"

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <surface_utils/smart_ptr.hpp>
#include <surface_utils/pcl_utils.hpp>

namespace surface_detection {

SurfaceDetection::SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist,
                                   double mls_radius, unsigned int min_pts_in_surface, double min_plane_width,
                                   double alpha, float extrusion_distance, std::string target_frame,
                                   std::string camera_frame)
    : target_frame_(target_frame), parallel_distance_(parallel_dist), perpendicular_distance_(perpendicular_dist),
      sqr_perpendicular_distance_(perpendicular_dist * perpendicular_dist),
      // State
      surfaces_(),
      // Implementation
      collect_points_(discretization, perpendicular_dist, target_frame, camera_frame),
      expand_surfaces_(perpendicular_dist, parallel_dist),
      detect_surfaces_(perpendicular_dist, parallel_dist, mls_radius, min_pts_in_surface, min_plane_width),
      build_surface_(perpendicular_dist, alpha, extrusion_distance) {}

SurfaceDetection::Surfaces SurfaceDetection::detect_surfaces_within(const Eigen::Affine3f &center,
                                                                    const Eigen::Vector3f &extents,
                                                                    const SurfaceVisualizationController &p) {
    pcl::ScopeTime st("SurfaceDetection::detect_surfaces_within");

    // Make an object to populate
    Surfaces new_surfaces;
    new_surfaces.header.frame_id = target_frame_;

    // Get points to process
    auto input = collect_points_.pending_points_within(center, extents);
    ROS_DEBUG_STREAM("Processing " << input.second.indices.size() << " points");
    p.pair("input_to_detection", input);

    // Get existing surfaces nearby and try to expand them
    collect_points_.surfaces_within(center, extents, [&, this](uint32_t surface_id) {
        const auto existing_surface = get_surface(surface_id);
        existing_surface.validate();
        this->find_merge(existing_surface, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
            if (merged.Valid()) {
                merged.Get().validate();
                ROS_DEBUG_STREAM(merged.Get().print_color()
                                 << "Merged existing surface " << merged.Get().id << " (" << merged.Get().inliers.size()
                                 << " points) with existing surface " << existing_surface.print_color() << "surface "
                                 << existing_surface.id << " (" << existing_surface.inliers.size() << " points)");
                this->remove_surface(existing_surface, p);
                ROS_WARN_STREAM(merged.Get().print_color()
                                << "Found a merged surface before expanding or detecting -- is that expected?");
                new_surfaces.update_surface(merged.Get());
            } else {
                ROS_DEBUG_STREAM(existing_surface.print_color() << "Added existing surface " << existing_surface.id);

                // Then this surface should be added with no modification
                new_surfaces.add_surface(existing_surface);
            }
        });
    });
    ROS_DEBUG_STREAM("Started with " << new_surfaces.surfaces.size() << " existing surfaces");

    auto indices_before = input.second.indices.size();
    input.second = expand_surfaces_.expand_surfaces(new_surfaces.surfaces, input, [&, this](Surface s_expanded) {
        s_expanded.validate();
        ROS_DEBUG_STREAM(s_expanded.print_color() << "Expanded existing surface " << s_expanded.id);
        this->find_merge(s_expanded, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
            if (merged.Valid()) {
                merged.Get().validate();
                ROS_DEBUG_STREAM(merged.Get().print_color() << "Merged expanded existing surface " << merged.Get().id
                                                            << " (" << merged.Get().inliers.size() << " points) into "
                                                            << s_expanded.print_color() << "existing surface "
                                                            << s_expanded.id << " (" << s_expanded.inliers.size()
                                                            << " points)");
                this->remove_surface(s_expanded, p);
                new_surfaces.update_surface(merged.Get());
            } else {
                // The surface was still changed even if it wasn't merged
                new_surfaces.update_surface(s_expanded);
            }
        });
    });
    ROS_DEBUG_STREAM("Expand surfaces reduced number of indices from " << indices_before << " to "
                                                                       << input.second.indices.size());

    // Build a search object for the input points
    pcl::search::KdTree<Point> search(false);
    search.setInputCloud(boost::shared_ptr<PointCloud>(&input.first, null_deleter()),
                         boost::shared_ptr<std::vector<int>>(&input.second.indices, null_deleter()));

    // Make some shorter names for the sake of formatting
    using Indices = pcl::PointIndices;

    // Detect new surfaces
    detect_surfaces_.detect_surfaces(input, p, [&](Indices indices, pcl::ModelCoefficients model, Eigen::Affine3f tf) {
        // Expect expand_new_surface to always find more points because of how detection works
        expand_surfaces_.expand_new_surface(input.first, search, indices, tf, [&, this](pcl::PointIndices in) {
            PointCloud new_inliers_cloud(input.first, in.indices);

            this->find_merge(new_inliers_cloud, model, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
                if (merged.Valid()) {
                    merged.Get().validate();
                    ROS_DEBUG_STREAM(merged.Get().print_color()
                                     << "Merged surface " << merged.Get().id << " (" << merged.Get().inliers.size()
                                     << " points) with un-numbered new surface (" << new_inliers_cloud.size()
                                     << " points)");
                    assert(merged.Get().model.values.size() == 4 &&
                           "Find merge returned a surface without a valid model");
                    new_surfaces.update_surface(merged.Get());
                } else {
                    Surface new_surface = build_surface_.new_partial_surface(new_inliers_cloud, model, tf);
                    ROS_DEBUG_STREAM(new_surface.print_color() << "Got a brand new surface with id " << new_surface.id);
                    new_surfaces.add_surface(new_surface);
                }
            });
        });
    });
    ROS_DEBUG_STREAM("Detected a total of " << new_surfaces.surfaces.size() << " surfaces");

    // Final pass through surfaces to turn all partial surfaces into complete surfaces
    for (const auto &surface : new_surfaces.surfaces) {
        if (!surface.is_complete()) {
            build_surface_.build_updated_surface(surface, p, [&, this](Surface updated_surface) {
                ROS_DEBUG_STREAM(updated_surface.print_color() << "Built updated surface " << updated_surface.id);
                this->add_or_update_surface(updated_surface, p);
                new_surfaces.update_surface(updated_surface);
            });
        }
    }

    return new_surfaces;
}

surface_types::Surface SurfaceDetection::get_surface(uint32_t surface_id) const {
    auto sit = surfaces_.find(surface_id);
    assert(sit != surfaces_.end() && "Tried to get a surface we don't have");
    return sit->second;
}

void SurfaceDetection::find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                                  const std::vector<Surface> &surfaces, const Maybe::Maybe<uint32_t> &ignore_surface,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
    assert(model.values.size() == 4 && "Find merge recieved invalid model");
    auto test_normal = Eigen::Map<const Eigen::Vector3f>(model.values.data());
    pcl::search::KdTree<Point> search(false);
    for (auto &surface : surfaces) {
        surface.validate();
        if (ignore_surface.Valid() && surface.id == ignore_surface.GetImmutable()) {
            continue;
        }

        auto candidate_normal = Eigen::Map<const Eigen::Vector3f>(surface.model.values.data());
        if (std::abs(std::acos(test_normal.dot(candidate_normal))) > 0.1) {
            // If the angle between the planes is greater than 1 radian, don't merge
            continue;
        }

        if (!search.getInputCloud()) {
            search.setInputCloud(boost::shared_ptr<const PointCloud>(&inliers, null_deleter()));
        }

        std::vector<int> indices;
        std::vector<float> distances;

        auto n_close = 0;
        for (auto &pt : surface.is_complete() ? surface.boundary : surface.inliers) {
            if (search.radiusSearch(pt, parallel_distance_, indices, distances, 1) > 0) {
                n_close += 1;

                if (n_close > 3) {
                    // Then merge!
                    Surface new_surface;
                    new_surface.id = surface.id;
                    new_surface.color = surface.color;
                    new_surface.model = surface.model; // Use the model of the original TODO: or the largest?
                    new_surface.pose = surface.pose;   // Use the pose of the original
                    new_surface.inliers = surface.inliers;
                    new_surface.inliers += inliers;

                    //                    ROS_INFO_STREAM(surface.print_color() << "Merging surface " << surface.id << "
                    //                    with "
                    //                                                          << inliers.size() << " new points");
                    callback(new_surface);
                    return; // Don't try to merge more than once
                }
            }
        }
    }

    // If we got here, then no merge needed
    callback({});
}

void SurfaceDetection::find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                                  const std::vector<Surface> &surfaces,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
    find_merge(inliers, model, surfaces, {}, callback);
}

void SurfaceDetection::find_merge(const Surface &test_surf, const std::vector<Surface> &surfaces,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
    test_surf.validate();
    find_merge(test_surf.inliers, test_surf.model, surfaces, test_surf.id, callback);
}

void SurfaceDetection::remove_surface(const Surface &surface, const SurfaceVisualizationController &p) {
    auto n_removed = surfaces_.erase(surface.id);
    assert(n_removed > 0 && "Tried to remove a surface that didn't exist");
    assert(n_removed == 1 && "Tried to remove one surface and accidentally removed multiple -- how??");
    collect_points_.remove_surface(surface.id);

    p.remove_marker("polygons", surface.id);
    p.remove_marker("mesh", surface.id);
    p.remove_marker("pose", surface.id);
    p.remove_marker("pose_x", surface.id);
    p.remove_marker("pose_y", surface.id);
}

void SurfaceDetection::add_or_update_surface(Surface &updated_surface, const SurfaceVisualizationController &p) {
    updated_surface.validate_complete();

    bool is_new = surfaces_.find(updated_surface.id) == surfaces_.end();
    surfaces_[updated_surface.id] = updated_surface;
    auto tiling = build_surface_.tile_surface(updated_surface);

    if (!is_new) {
        // Remove previous points
        collect_points_.remove_surface(updated_surface.id);
    }
    collect_points_.add_surface(tiling, updated_surface.id);

    p.points<Point>("tiling", tiling.makeShared());
    p.mesh("mesh", updated_surface);
    p.polygons("polygons", updated_surface);
    p.pose("pose", updated_surface);
}
}