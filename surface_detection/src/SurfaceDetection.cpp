//
// Created by will on 6/29/16.
//

#include "surface_detection/SurfaceDetection.h"

#include <surface_types/Surface.hpp>

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <surface_utils/smart_ptr.hpp>
#include <surface_utils/pcl_utils.hpp>
#include <eigen_conversions/eigen_kdl.h>

namespace surface_detection {

SurfaceDetection::SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist,
                                   double point_inside_threshold, double mls_radius, unsigned int min_pts_in_surface,
                                   double min_plane_width, double alpha, float extrusion_distance, bool optimistic,
                                   std::string target_frame, std::string camera_frame)
    : target_frame_(target_frame), discretization_(discretization), parallel_distance_(parallel_dist),
      perpendicular_distance_(perpendicular_dist), min_width_(min_plane_width),
      sqr_perpendicular_distance_(perpendicular_dist * perpendicular_dist), min_pts_in_surface_(min_pts_in_surface),
      optimistic_(optimistic),
      // State
      surfaces_(),
      // Delegation
      collect_points_(discretization, perpendicular_dist, point_inside_threshold, target_frame, camera_frame),
      expand_surfaces_(perpendicular_dist, parallel_dist, discretization),
      detect_surfaces_(perpendicular_dist, parallel_dist, mls_radius, min_pts_in_surface, min_plane_width),
      build_surface_(perpendicular_dist, parallel_dist, point_inside_threshold, alpha, extrusion_distance) {}

void SurfaceDetection::add_start_surface(float start_surface_extent_x, float start_surface_extent_y,
                                         const SurfaceVisualizationController &v) {
    pcl::ModelCoefficients coeff;
    coeff.values = {0, 0, 1, 0};
    PointCloud pts;
    for (float x = -start_surface_extent_x; x < start_surface_extent_x; x += discretization_) {
        for (float y = -start_surface_extent_y; y < start_surface_extent_y; y += discretization_) {
            Point pt;
            pt.x = x;
            pt.y = y;
            pts.push_back(pt);
        }
    }

    Surface start_surface(std::move(pts), std::move(coeff), Eigen::Affine3d::Identity());
    start_surface.update(build_surface_.compute_derived(start_surface, v));

    this->add_or_update_surface(start_surface, v);
    expansion_skipped_.insert(start_surface.id());
}

SurfaceDetection::Surfaces SurfaceDetection::detect_surfaces_within(const Eigen::Affine3f &center,
                                                                    const Eigen::Vector3f &extents,
                                                                    const SurfaceVisualizationController &v) {
    pcl::ScopeTime st("SurfaceDetection::detect_surfaces_within");

    float roll, pitch, yaw;
    pcl::getEulerAngles(center, roll, pitch, yaw);
    ROS_DEBUG_STREAM("Detecting surfaces within " << extents.transpose() << " of " << center.translation().transpose()
                                                  << ", orientation " << roll << ", " << pitch << ", " << yaw);

    // Make the objects to store working data
    std::set<uint32_t> collected_surfaces;
    PointCloud all_pts;
    std::vector<int> considered_indices;
    std::vector<std::vector<int>> indices_to_remove;

    wait_for_cleanup();

    // Fairly complicated C++11-y statement to populate all_pts and considered_indices, and to save the first list
    // of indices that will be removed (because they are within the non-padded query box)
    std::forward_as_tuple(std::tie(all_pts, considered_indices), std::back_inserter(indices_to_remove)) =
        collect_points_.pending_points_within(center, extents, static_cast<float>(min_width_));
    ROS_DEBUG_STREAM("Processing " << considered_indices.size() << " points out of " << all_pts.size());
    v.points("input_to_detection", all_pts, considered_indices);

    // Get existing surfaces nearby
    collect_points_.surfaces_within(center, extents, [&, this](uint32_t surface_id) {
        ROS_DEBUG_STREAM(get_surface(surface_id).print_color() << "Added existing surface " <<
                                 get_surface(surface_id).id());
        collected_surfaces.insert(surface_id);
    });
    ROS_DEBUG_STREAM("Started with " << collected_surfaces.size() << " existing surfaces");

#if 0
    // Expand existing surfaces, if needed
    // Note that it is intentional that there are two expansion phases, because the first removes points that then do
    // not need to be considered by detect surfaces, which happens in between.
    if (optimistic_) {
        ROS_DEBUG_STREAM("Optimistic mode does not require expanding existing surfaces");
    } else if (new_surfaces.surfaces.size() == 0) {
        ROS_DEBUG_STREAM("Not doing expansion because there are no surfaces");
    } else {
        auto n_indices_before = considered_indices.size();
        // Pessimistic mode: only expand as far as the (padded) query box
        // This call removes the points it adds to a surface from considered_indices
        expand_surfaces_.expand_surfaces(new_surfaces.surfaces, all_pts, considered_indices, [&, this](Surface new_s) {
            new_s.validate();
            ROS_DEBUG_STREAM(new_s.print_color() << "Expanded existing surface " << new_s.id << " by "
                                                 << new_s.inliers.size() - surfaces_[new_s.id].inliers.size()
                                                 << " points");
            this->find_merge(new_s, new_surfaces.surfaces, [&, this](Maybe::Maybe<Surface> merged) {
                if (merged.Valid()) {
                    merged.Get().validate();
                    ROS_DEBUG_STREAM(merged.Get().print_color()
                                     << "Merged expanded existing surface " << merged.Get().id << " ("
                                     << merged.Get().inliers.size() << " points) into " << new_s.print_color()
                                     << "existing surface " << new_s.id << " (" << new_s.inliers.size() << " points)");
                    this->remove_surface(new_s, v);
                    new_surfaces.update_surface(merged.Get());
                } else {
                    // The surface was still changed even if it wasn't merged
                    new_surfaces.update_surface(new_s);
                }
            });
        });
        ROS_DEBUG_STREAM("Expand surfaces reduced number of indices from " << n_indices_before << " to "
                                                                           << considered_indices.size());
    }

    if ((optimistic_ ? all_pts.size() : considered_indices.size()) < 3) {
        ROS_DEBUG_STREAM("Not enough points in the expansion cloud to expand new surfaces");
    } else {
        std::vector<Surface> surfaces_to_expand;

        // This conditional is a very, very dirty hack to avoid the unrepresentative expansion step on baseline
        // (The logic inside is not a hack)
        if (!expansion_skipped_.empty() && (extents.array() < 4.f).all()) {
            for (const auto existing_surface : new_surfaces.surfaces) {
                auto found = expansion_skipped_.find(existing_surface.getId());
                if (found != expansion_skipped_.end()) {
                    surfaces_to_expand.push_back(existing_surface);
                    expansion_skipped_.erase(found);
                }
            }

            ROS_DEBUG_STREAM("Expanding " << surfaces_to_expand.size()
                                          << " existing surfaces whose expansion was skipped");
        }

        std::vector<int> used_points;
        if (considered_indices.size() < min_pts_in_surface_) {
            ROS_DEBUG_STREAM("Not enough remaining indices to try to find new surfaces (" << considered_indices.size()
                                                                                          << ")");
        } else {
            // Detect new surfaces (mutates considered_indices to remove all points that were added to a new surface)
            detect_surfaces_.detect_surfaces(all_pts, considered_indices, v,
                [&](std::vector<int> inliers, pcl::ModelCoefficients model, Eigen::Affine3f tf) {
                    surfaces_to_expand.push_back(build_surface_.new_partial_surface({all_pts, inliers}, model, tf));
                });
        }

        if (!surfaces_to_expand.empty()) {
            expand_surfaces_.expand_surfaces(surfaces_to_expand, all_pts, ???, [&](Surface expanded) {
                // VERY MUCH TODO: Need to pass optimistic_ ? (all_indices(cloud) - used) : considered_indices to this
            });












            auto kdtree_start = ros::WallTime::now();
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
            ROS_DEBUG_STREAM("Making kdtree took " << (ros::WallTime::now() - kdtree_start).toSec() << "s");

            // Expand new surfaces
            for (auto ds : surfaces_to_expand) {
                // Expect expand_new_surface to always find more points because detection always discouts edge points
                expand_surfaces_.expand_new_surface(input.first, search, ds, [&, this](pcl::PointIndices in) {
                    ROS_DEBUG_STREAM("Expanding new surfaces added " << in.indices.size() - ds.inliers.size()
                                                                     << " new points");
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
            build_surface_.build_updated_surface(surface, v, [&, this](Surface updated_surface) {
                ROS_DEBUG_STREAM(updated_surface.print_color() << "Built new or updated surface "
                                                               << updated_surface.getId());
                new_surfaces.update_surface(updated_surface);
                needs_update.push_back(updated_surface);
            });
        }
    }

    cleanup_done_ = std::async(
        std::launch::async,
        std::bind([this, &v](std::vector<Surface> &to_update, PointCloud &rm_pts, std::vector<int> &rm_indices) {
            ROS_DEBUG_STREAM_NAMED("cleanup", "Running cleanup");
            collect_points_.remove_voxels_at_points(rm_pts, rm_indices);
            ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of points inside query box");
            for (auto &surface : to_update) {
                add_or_update_surface(surface, v);
                ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of points inside surface " << surface.id());
            }
            ROS_DEBUG_STREAM_NAMED("cleanup", "Cleanup done");
        }, std::move(needs_update), std::move(input.first), std::move(indices_to_remove)));
#endif

    // Make an object to populate
    Surfaces new_surfaces;
    new_surfaces.header.frame_id = target_frame_;


    return new_surfaces;
}

void SurfaceDetection::wait_for_cleanup() const {
    if (cleanup_done_.valid()) {
        auto start = std::chrono::steady_clock::now();
        cleanup_done_.wait();
        auto time = std::chrono::steady_clock::now() - start;
        if (time > std::chrono::milliseconds(50)) {
            ROS_DEBUG_STREAM("Waited " << std::chrono::duration_cast<std::chrono::milliseconds>(time).count()
                                       << "ms for the previous run's cleanup");
        }
    }
}

Surface SurfaceDetection::get_surface(uint32_t surface_id) const {
    auto sit = surfaces_.find(surface_id);
    assert(sit != surfaces_.end() && "Tried to get a surface we don't have");
    return sit->second;
}

void SurfaceDetection::find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                                  const std::vector<Surface> &surfaces, const Maybe::Maybe<uint32_t> &ignore_surface,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
//    assert(model.values.size() == 4 && "Find merge recieved invalid model");
//    auto test_normal = Eigen::Map<const Eigen::Vector3f>(model.values.data());
//    pcl::search::KdTree<Point> search(false);
//    for (auto &surface : surfaces) {
//        if (ignore_surface.Valid() && surface.id() == ignore_surface.GetImmutable()) {
//            continue;
//        }
//
//        auto candidate_normal = Eigen::Map<const Eigen::Vector3f>(surface.model().values.data());
//        if (std::abs(std::acos(test_normal.dot(candidate_normal))) > (5. * M_PI / 180.)) {
//            // If the angle between the planes is greater than a given angle, don't merge
//            continue;
//        }
//
//        if (std::abs(model.values[3] - surface.model().values[3]) > perpendicular_distance_) {
//            // If the surfaces are too far away, don't merge
//            continue;
//        }
//
//        if (!search.getInputCloud()) {
//            search.setInputCloud(boost::shared_ptr<const PointCloud>(&inliers, null_deleter()));
//        }
//
//        std::vector<int> indices;
//        std::vector<float> distances;
//
//        auto n_close = 0;
//        for (auto &pt : surface.boundary_or_inliers()) {
//            if (search.radiusSearch(pt, parallel_distance_, indices, distances, 1) > 0) {
//                n_close += 1;
//
//                if (n_close > 3) {
//                    // Then merge!
//                    Surface new_surface = surface;
//                    new_surface.add_inliers(inliers);
//                    callback(new_surface);
//                    return; // Don't try to merge more than once
//                }
//            }
//        }
//    }
//
//    // If we got here, then no merge needed
//    callback({});
}

void SurfaceDetection::find_merge(const PointCloud &inliers, const pcl::ModelCoefficients &model,
                                  const std::vector<Surface> &surfaces,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
    find_merge(inliers, model, surfaces, {}, callback);
}

void SurfaceDetection::find_merge(const Surface &test_surf, const std::vector<Surface> &surfaces,
                                  const std::function<void(Maybe::Maybe<Surface>)> &callback) {
    find_merge(test_surf.inliers(), test_surf.model(), surfaces, test_surf.id(), callback);
}

void SurfaceDetection::remove_surface(const Surface &surface, const SurfaceVisualizationController &p) {
    auto n_removed = surfaces_.erase(surface.id());
    assert(n_removed > 0 && "Tried to remove a surface that didn't exist");
    assert(n_removed == 1 && "Tried to remove one surface and accidentally removed multiple -- how??");
    collect_points_.remove_surface(surface.id());

    p.remove_marker("polygons", surface.id());
    p.remove_marker("mesh", surface.id());
    p.remove_marker("pose", surface.id());
    p.remove_marker("pose_x", surface.id());
    p.remove_marker("pose_y", surface.id());
}

void SurfaceDetection::add_or_update_surface(Surface &updated_surface, const SurfaceVisualizationController &p) {
    bool is_new = surfaces_.find(updated_surface.id()) == surfaces_.end();
    surfaces_.emplace(updated_surface.id(), updated_surface);

    auto tiling = build_surface_.tile_surface(updated_surface);

    if (!is_new) {
        // Remove previous points
        collect_points_.remove_surface(updated_surface.id());
    }
    collect_points_.add_surface(tiling, updated_surface.id());

    p.points<Point>("tiling", tiling.makeShared());
    p.mesh("mesh", updated_surface);
    p.polygons("polygons", updated_surface);
    p.pose("pose", updated_surface);
    p.plane_normal("normal", updated_surface);
}

size_t SurfaceDetection::get_num_pending_points() { return collect_points_.num_pending_points(); }
}