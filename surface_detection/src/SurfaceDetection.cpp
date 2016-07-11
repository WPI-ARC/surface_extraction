//
// Created by will on 6/29/16.
//

#include "surface_detection/SurfaceDetection.h"

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <surface_utils/smart_ptr.hpp>
#include <surface_utils/pcl_utils.hpp>
#include <eigen_conversions/eigen_kdl.h>

namespace surface_detection {

SurfaceDetection::SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist,
                                   double point_inside_threshold, double mls_radius, unsigned int min_pts_in_surface,
                                   double min_plane_width, double alpha, float extrusion_distance,
                                   std::string target_frame, std::string camera_frame)
    : target_frame_(target_frame), parallel_distance_(parallel_dist), perpendicular_distance_(perpendicular_dist), min_width_(min_plane_width),
      sqr_perpendicular_distance_(perpendicular_dist * perpendicular_dist), min_pts_in_surface_(min_pts_in_surface),
      // State
      surfaces_(),
      // Implementation
      collect_points_(discretization, perpendicular_dist, point_inside_threshold, target_frame, camera_frame),
      expand_surfaces_(perpendicular_dist, parallel_dist, discretization),
      detect_surfaces_(perpendicular_dist, parallel_dist, mls_radius, min_pts_in_surface, min_plane_width),
      build_surface_(perpendicular_dist, parallel_dist, point_inside_threshold, alpha, extrusion_distance) {}

void SurfaceDetection::add_start_surface(double discretization, double start_surface_extent_x,
                                         double start_surface_extent_y, const SurfaceVisualizationController &p) {
    pcl::ModelCoefficients coeff;
    coeff.values = {0, 0, 1, 0};
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    PointCloud pts;
    for (float x = -static_cast<float>(start_surface_extent_x); x < start_surface_extent_x; x += discretization) {
        for (float y = -static_cast<float>(start_surface_extent_y); y < start_surface_extent_y; y += discretization) {
            Point pt;
            pt.x = x;
            pt.y = y;
            pts.push_back(pt);
        }
    }
    build_surface_.build_new_surface(std::move(pts), std::move(coeff), std::move(pose), p,
                                     [&, this](Surface updated_surface) {
                                         this->add_or_update_surface(updated_surface, p);
                                         expansion_skipped_.insert(updated_surface.id);
                                     });
}

SurfaceDetection::Surfaces SurfaceDetection::detect_surfaces_within(const Eigen::Affine3f &center,
                                                                    const Eigen::Vector3f &extents,
                                                                    const SurfaceVisualizationController &p) {
    pcl::ScopeTime st("SurfaceDetection::detect_surfaces_within");
    bool optimistic = true;  // TODO parameter

    float roll, pitch, yaw;
    pcl::getEulerAngles(center, roll, pitch, yaw);
    ROS_DEBUG_STREAM("Detecting surfaces within " << extents.transpose() << " of " << center.translation().transpose()
                                                  << ", orientation " << roll << ", " << pitch << ", " << yaw);

    // Make an object to populate
    Surfaces new_surfaces;
    new_surfaces.header.frame_id = target_frame_;

    if (cleanup_done_.valid()) {
        auto start = std::chrono::steady_clock::now();
        cleanup_done_.wait();
        auto time = std::chrono::steady_clock::now() - start;
        //        if (time > std::chrono::milliseconds(50)) {
        ROS_DEBUG_STREAM("Waited " << std::chrono::duration_cast<std::chrono::milliseconds>(time).count()
                                   << "ms for the previous run's cleanup");
        //        }
    }

    // Get points to process
    CloudIndexPair input;
    std::vector<int> indices_to_remove;
    std::tie(input, indices_to_remove) = collect_points_.pending_points_within(center, extents, min_width_);
    ROS_DEBUG_STREAM("Processing " << input.second.indices.size() << " points");
    p.pair("input_to_detection", input);

    // Get existing surfaces nearby and try to expand them
    collect_points_.surfaces_within(center, extents, [&, this](uint32_t surface_id) {
        const auto existing_surface = get_surface(surface_id);
        existing_surface.validate();
        //        this->find_merge(existing_surface, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
        //            if (merged.Valid()) {
        //                merged.Get().validate();
        //                ROS_DEBUG_STREAM(merged.Get().print_color()
        //                                 << "Merged existing surface " << merged.Get().id << " (" <<
        //                                 merged.Get().inliers.size()
        //                                 << " points) with existing surface " << existing_surface.print_color() <<
        //                                 "surface "
        //                                 << existing_surface.id << " (" << existing_surface.inliers.size() << "
        //                                 points)");
        //                this->remove_surface(existing_surface, p);
        //                ROS_WARN_STREAM(merged.Get().print_color()
        //                                << "Found a merged surface before expanding or detecting -- is that
        //                                expected?");
        //                new_surfaces.update_surface(merged.Get());
        //            } else {
        ROS_DEBUG_STREAM(existing_surface.print_color() << "Added existing surface " << existing_surface.id);

        // Then this surface should be added with no modification
        new_surfaces.add_surface(existing_surface);
        //            }
        //        });
    });
    ROS_DEBUG_STREAM("Started with " << new_surfaces.surfaces.size() << " existing surfaces");

    if (optimistic) {
        ROS_DEBUG_STREAM("Optimistic mode does not require expanding existing surfaces");
    } else if (new_surfaces.surfaces.size() == 0) {
        ROS_DEBUG_STREAM("Not doing expansion because there are no surfaces");
    } else if (input.second.indices.size() < 3) {
        ROS_DEBUG_STREAM("Not enough points to do expansion (" << input.second.indices.size() << ")");
    } else {
        auto indices_before = input.second.indices.size();
        input.second = expand_surfaces_.expand_surfaces(new_surfaces.surfaces, input, [&, this](Surface s_expanded) {
            s_expanded.validate();
            ROS_DEBUG_STREAM(s_expanded.print_color()
                             << "Expanded existing surface " << s_expanded.id << " by "
                             << s_expanded.inliers.size() - surfaces_[s_expanded.id].inliers.size() << " points");
            this->find_merge(s_expanded, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
                if (merged.Valid()) {
                    merged.Get().validate();
                    ROS_DEBUG_STREAM(merged.Get().print_color()
                                     << "Merged expanded existing surface " << merged.Get().id << " ("
                                     << merged.Get().inliers.size() << " points) into " << s_expanded.print_color()
                                     << "existing surface " << s_expanded.id << " (" << s_expanded.inliers.size()
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
    }

    if (input.first.size() < 3) {
        ROS_DEBUG_STREAM("Not enough points in the entire pending points cloud to expand new surfaces");
    } else {
        std::vector<Surface> surfaces_to_expand;

        if (!expansion_skipped_.empty()) {
            for (const auto existing_surface : new_surfaces.surfaces) {
                auto found = expansion_skipped_.find(existing_surface.id);
                if (found != expansion_skipped_.end()) {
                    surfaces_to_expand.push_back(existing_surface);
                    expansion_skipped_.erase(found);
                }
            }

            ROS_DEBUG_STREAM("Expanding " << surfaces_to_expand.size()
                                          << " existing surfaces whose expansion was skipped");
        }

        std::vector<int> used_points;
        if (input.second.indices.size() < min_pts_in_surface_) {
            ROS_DEBUG_STREAM("Not enough remaining indices to try to find new surfaces (" << input.second.indices.size() << ")");
        } else {
            // Make some shorter names for the sake of formatting
            using Indices = pcl::PointIndices;
            using Model = pcl::ModelCoefficients;

            // Detect new surfaces
            used_points =
                detect_surfaces_.detect_surfaces(input, p, [&](Indices indices, Model model, Eigen::Affine3f tf) {
                    PointCloud new_inliers_cloud(input.first, indices.indices);
                    surfaces_to_expand.push_back(build_surface_.new_partial_surface(new_inliers_cloud, model, tf));
                });
            ROS_DEBUG_STREAM("Detected a total of " << new_surfaces.surfaces.size() << " surfaces");
        }

        if (!surfaces_to_expand.empty()) {
            // If optimistic, expand into all the points; otherwise, only the ones in the given bounding box
            auto all_indices = optimistic ? surfaces_pcl_utils::all_indices(input.first) : input.second.indices;
            std::sort(all_indices.begin(), all_indices.end());
            std::sort(used_points.begin(), used_points.end());

            std::vector<int> indices_for_expansion;
            indices_for_expansion.reserve(all_indices.size() - used_points.size());
            std::set_difference(all_indices.begin(), all_indices.end(), used_points.begin(), used_points.end(),
                                std::back_inserter(indices_for_expansion));

            // Build a search object for the input points
            pcl::search::KdTree<Point> search(false);
            search.setInputCloud(boost::shared_ptr<PointCloud>(&input.first, null_deleter()),
                                 boost::shared_ptr<std::vector<int>>(&indices_for_expansion, null_deleter()));

            // Expand new surfaces
            for (auto ds : surfaces_to_expand) {
                // Expect expand_new_surface to always find more points because of how detection works
                expand_surfaces_.expand_new_surface(input.first, search, ds, [&, this](pcl::PointIndices in) {
                    ds.inliers += PointCloud(input.first, in.indices);

                    this->find_merge(ds, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
                        if (merged.Valid()) {
                            merged.Get().validate();
                            ROS_DEBUG_STREAM(merged.Get().print_color()
                                             << "Merged surface " << merged.Get().id << " ("
                                             << merged.Get().inliers.size() << " points) with " << ds.print_color()
                                             << "new surface " << ds.id << " (" << ds.inliers.size() << " points)");
                            assert(merged.Get().model.values.size() == 4 &&
                                   "Find merge returned a surface without a valid model");
                            new_surfaces.update_surface(merged.Get());
                        } else {
                            ROS_DEBUG_STREAM(ds.print_color() << "Got a brand new surface with id " << ds.id);
                            new_surfaces.add_surface(ds);
                        }
                    });
                });
            }
        }
    }
    std::vector<Surface> needs_update;

    // Final pass through surfaces to turn all partial surfaces into complete surfaces
    for (const auto &surface : new_surfaces.surfaces) {
        if (!surface.is_complete()) {
            build_surface_.build_updated_surface(surface, p, [&, this](Surface updated_surface) {
                ROS_DEBUG_STREAM(updated_surface.print_color() << "Built new or updated surface "
                                                               << updated_surface.id);
                new_surfaces.update_surface(updated_surface);
                needs_update.push_back(updated_surface);
            });
        }
    }

    cleanup_done_ = std::async(
        std::launch::async,
        std::bind([this, &p](std::vector<Surface> &to_update, PointCloud &rm_pts, std::vector<int> &rm_indices) {
            ROS_DEBUG_STREAM_NAMED("cleanup", "Running cleanup");
            collect_points_.remove_voxels_at_points(rm_pts, rm_indices);
            ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of points inside query box");
            for (auto &surface : to_update) {
                add_or_update_surface(surface, p);
                ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of points inside surface " << surface.id);
            }
            ROS_DEBUG_STREAM_NAMED("cleanup", "Cleanup done");
        }, std::move(needs_update), std::move(input.first), std::move(indices_to_remove)));

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
        if (std::abs(std::acos(test_normal.dot(candidate_normal))) > (5. * M_PI / 180.)) {
            // If the angle between the planes is greater than a given angle, don't merge
            continue;
        }

        if (std::abs(model.values[3] - surface.model.values[3]) > perpendicular_distance_) {
            // If the surfaces are too far away, don't merge
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

                    // TODO Integrate mesh-reuse much better
                    new_surface.inliers_at_last_computation_ = surface.inliers_at_last_computation_;
                    new_surface.mesh = surface.mesh;

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
    p.plane_normal("normal", updated_surface);
}
}