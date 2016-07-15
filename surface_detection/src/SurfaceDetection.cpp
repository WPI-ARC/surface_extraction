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

#define TIME_FMT(amt, total, msg)                                                                                      \
    ROS_DEBUG_STREAM("    -" << std::fixed << std::setprecision(0) << std::setw(3)                                     \
                             << (amt.count() / total.count() * 100) << "%, " << std::setw(5)                           \
                             << duration_cast<milliseconds>(amt).count() << "ms: " msg)

namespace surface_detection {

using namespace std::chrono;

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

SurfaceDetection::RGBCloud SurfaceDetection::get_surface_points() {
    auto labeled_cloud = collect_points_.get_surface_points();
    RGBCloud rgb_cloud;
    for (auto &pt : labeled_cloud) {
        PointRGB rgb_pt;
        rgb_pt.x = pt.x;
        rgb_pt.y = pt.y;
        rgb_pt.z = pt.z;
        rgb_pt.r = static_cast<uint8_t>(get_surface(pt.label).color().r * 255);
        rgb_pt.g = static_cast<uint8_t>(get_surface(pt.label).color().g * 255);
        rgb_pt.b = static_cast<uint8_t>(get_surface(pt.label).color().b * 255);
        rgb_cloud.push_back(rgb_pt);
    }
    return rgb_cloud;
}

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

    add_surface(std::move(start_surface));
    publish_updated_surfaces({start_surface.id()}, v);
    update_collector_surfaces();
    expansion_skipped_.insert(start_surface.id());
}

SurfaceDetection::Surfaces SurfaceDetection::detect_surfaces_within(const Eigen::Affine3f &center,
                                                                    const Eigen::Vector3f &extents,
                                                                    const Surface::ProvideLevel provide_inliers,
                                                                    const Surface::ProvideLevel provide_shape,
                                                                    const Surface::ProvideLevel provide_mesh,
                                                                    const SurfaceVisualizationController &v) {
    duration<double> t_total, t_wait, t_pending_points, t_surfaces_within, t_return;
    auto t_total_start = high_resolution_clock::now();

    float roll, pitch, yaw;
    pcl::getEulerAngles(center, roll, pitch, yaw);
    ROS_DEBUG_STREAM("Detecting surfaces within " << extents.transpose() << " of " << center.translation().transpose()
                                                  << ", orientation " << roll << ", " << pitch << ", " << yaw);

    // Make the objects to store working data
    std::set<uint32_t> collected_surfaces;
    PointCloud all_pts;
    std::vector<int> considered_indices;
    std::vector<int> bb_indices;
    std::vector<uint32_t> deleted_surfaces;

    auto t_wait_start = high_resolution_clock::now();
    wait_for_cleanup();
    t_wait = high_resolution_clock::now() - t_wait_start;

    // Fairly magic C++11-y statement to populate all_pts and considered_indices, and to save the indices that will
    // be removed (because they are within the non-padded query box)
    auto t_pending_points_start = high_resolution_clock::now();
    std::forward_as_tuple(std::tie(all_pts, considered_indices), bb_indices) =
        collect_points_.pending_points_within(center, extents, static_cast<float>(min_width_));
    ROS_DEBUG_STREAM("Processing " << considered_indices.size() << " points out of " << all_pts.size());
    v.points("input_to_detection", all_pts, considered_indices);
    t_pending_points = high_resolution_clock::now() - t_pending_points_start;

    std::vector<int> new_labels(all_pts.size(), -1);

    // Get existing surfaces nearby
    auto t_surfaces_within_start = high_resolution_clock::now();
    collect_points_.surfaces_within(center, extents, [&, this](uint32_t surface_id) {
        ROS_DEBUG_STREAM(get_surface(surface_id).print_color() << "Added existing surface "
                                                               << get_surface(surface_id).id());
        collected_surfaces.insert(surface_id);
    });
    ROS_DEBUG_STREAM("Started with " << collected_surfaces.size() << " existing surfaces");
    t_surfaces_within = high_resolution_clock::now() - t_surfaces_within_start;

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
#endif

    cleanup_done_ = std::async(std::launch::async, std::bind(&SurfaceDetection::cleanup, this, std::move(all_pts),
                                                             std::move(deleted_surfaces), std::move(bb_indices),
                                                             std::move(new_labels), std::cref(v)));

    // Make an object to populate
    Surfaces new_surfaces;
    new_surfaces.header.frame_id = target_frame_;

    auto t_return_start = high_resolution_clock::now();
    for (auto &sid : collected_surfaces) {
        auto sit = surfaces_.find(sid);
        assert(sit != surfaces_.end() && "An id in collected_surface didn't match any surfaces in surfaces_");
        auto &surface = sit->second;

        // It's not possible for inliers to be out of date
        bool get_inliers = (provide_inliers != Surface::NEVER);
        // It is possible for the others to be out of date
        bool get_shape =
            (provide_shape == Surface::ALWAYS || (provide_shape == Surface::IF_AVAILABLE && surface.has_shape()));
        bool get_mesh =
            (provide_mesh == Surface::ALWAYS || (provide_mesh == Surface::IF_AVAILABLE && surface.has_mesh()));

        // Make sure all necessary data is actually avalible
        assert(surface.has_plane() && "Surfaces should always have an updated plane at the end of detect_surfaces");
        if (get_shape && !surface.has_shape()) {
            ROS_DEBUG_STREAM(surface.print_color() << "Updating surface " << surface.id()
                                                   << "' s shape before returning");
            surface.update_shape(build_surface_.compute_shape(surface, v));
        }
        if (get_mesh && !surface.has_mesh()) {
            ROS_DEBUG_STREAM(surface.print_color() << "Updating surface " << surface.id()
                                                   << "' s mesh before returning");
            surface.update_mesh(build_surface_.compute_mesh(surface));
        }

        ROS_DEBUG_STREAM(surface.print_color() << "Returning surface " << surface.id());
        new_surfaces.surfaces.push_back(surface.get_data(get_inliers, get_shape, get_mesh));
    }
    t_return = high_resolution_clock::now() - t_return_start;

    t_total = high_resolution_clock::now() - t_total_start;
    duration<double> t_other = t_total - (t_wait + t_pending_points + t_surfaces_within + t_return);
    ROS_DEBUG_STREAM("Time breakdown of surface detection:");
    TIME_FMT(t_wait, t_total, "waiting for previous run's cleanup");
    TIME_FMT(t_pending_points, t_total, "getting pending points");
    TIME_FMT(t_surfaces_within, t_total, "getting existing surfaces");
    TIME_FMT(t_return, t_total, "building return object");
    TIME_FMT(t_other, t_total, "other");

    return new_surfaces;
}

void SurfaceDetection::cleanup(PointCloud &processed_pts, std::vector<uint32_t> &deleted, std::vector<int> &rm_indices,
                               std::vector<int> labels, const SurfaceVisualizationController &v) {
    ROS_DEBUG_STREAM_NAMED("cleanup", "Running cleanup");
    std::vector<uint32_t> updated;
    for (const auto &s : surfaces_) {
        if (s.second.needs_cleanup()) {
            updated.push_back(s.second.id());
        }
    }
    ROS_DEBUG_STREAM_NAMED("cleanup", "Cleaning up " << updated.size() << " updated surfaces and " << deleted.size()
                                                     << " deleted ones");

    if (!updated.empty()) {
        // If updated is empty, we know for sure that no indices were assigned to surfaces
        for (std::size_t i = 0; i < labels.size(); i++) {
            if (labels[i] >= 0) {
                // If this has a (nonnegative) label, that means it was assigned to a surface
                rm_indices.push_back(static_cast<int>(i));
            }
        }
    }
    collect_points_.remove_voxels_at_points(processed_pts, rm_indices);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of new inliers and points inside query box");

    // If no surfaces updated or deleted, there's no more work to do
    if (updated.empty() && deleted.empty()) return;

    update_collector_surfaces();
    ROS_DEBUG_STREAM_NAMED("cleanup", "Updated surfaces octree");

    publish_deleted_surfaces(deleted, v);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Deleted markers of deleted surfaces");
    publish_updated_surfaces(updated, v);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Updated markers of updated surfaces");
}

void SurfaceDetection::wait_for_cleanup() const {
    if (cleanup_done_.valid()) {
        auto start = high_resolution_clock::now();
        cleanup_done_.wait();
        duration<double> time = high_resolution_clock::now() - start;
        if (time > milliseconds(50)) {
            ROS_DEBUG_STREAM("Waited " << duration_cast<milliseconds>(time).count()
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

void SurfaceDetection::remove_surface(const Surface &surface) {
    auto n_removed = surfaces_.erase(surface.id());
    assert(n_removed > 0 && "Tried to remove a surface that didn't exist");
    assert(n_removed == 1 && "Tried to remove one surface and accidentally removed multiple -- how??");
    collect_points_.remove_surface(surface.id());
}

void SurfaceDetection::publish_deleted_surfaces(const std::vector<uint32_t> &ids,
                                                const SurfaceVisualizationController &v) const {
    for (auto id : ids) {
        v.remove_marker("pose", id);
        v.remove_marker("normal", id);
        v.remove_marker("pose_x", id);
        v.remove_marker("pose_y", id);
        v.remove_marker("polygons", id);
        v.remove_marker("mesh", id);
    }
}

void SurfaceDetection::publish_updated_surfaces(const std::vector<uint32_t> &ids,
                                                const SurfaceVisualizationController &v) const {
    for (auto id : ids) {
        const auto &surface = get_surface(id);
        if (surface.has_plane()) {
            v.pose("pose", surface);
            v.plane_normal("normal", surface);
        }
        if (surface.has_shape()) {
            v.polygons("polygons", surface);
        }
        if (surface.has_mesh()) {
            v.mesh("mesh", surface);
        }
    }
}

void SurfaceDetection::update_collector_surfaces() {
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
    for (auto sit : surfaces_) {
        auto &surface = sit.second;

        if (surface.needs_cleanup()) {
            surface.update_tiling(build_surface_.tile_surface(surface));
        }

        *cloud += surface.tiling();
    }

    ROS_DEBUG_STREAM("Updating surfaces with " << cloud->size() << " points from " << surfaces_.size() << " surfaces");
    collect_points_.update_surfaces(std::move(cloud));
}

size_t SurfaceDetection::get_num_pending_points() { return collect_points_.num_pending_points(); }

void SurfaceDetection::add_surface(Surface surface) {
    auto result = surfaces_.emplace(surface.id(), std::move(surface));
    assert(result.second && "Tried to insert a duplicate surface");
}
}