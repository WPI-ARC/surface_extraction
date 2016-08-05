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
#include <pcl/sample_consensus/sac_model_plane.h>

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
    for (auto &pt : *labeled_cloud) {
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
    std::promise<void> should_have_been_a_mutex;
    cleanup_done_ = should_have_been_a_mutex.get_future();

    if (start_surface_extent_x < min_width_ || start_surface_extent_y < min_width_) {
        ROS_DEBUG_STREAM("Not adding start surface because it is smaller than " << min_width_ << "m");
        return;
    }

    pcl::ModelCoefficients coeff;
    coeff.values = {0, 0, 1, 0};
    PointCloud pts;
    for (float x = -start_surface_extent_x; x <= start_surface_extent_x; x += discretization_) {
        for (float y = -start_surface_extent_y; y <= start_surface_extent_y; y += discretization_) {
            Point pt;
            pt.x = x;
            pt.y = y;
            pts.push_back(pt);
        }
    }

    Surface start_surface(std::move(pts), std::move(coeff), Eigen::Affine3d::Identity());
    start_surface.update(build_surface_.compute_derived(start_surface, v));

    auto id = start_surface.id();
    add_surface(std::move(start_surface));
    publish_updated_surfaces({id}, v);
    update_collector_surfaces(v);
    expansion_skipped_.insert(id);

    cleanup(PointCloud(), {id}, {}, {}, {}, v);

    // "unlock" the "mutex"
    should_have_been_a_mutex.set_value();
}

SurfaceDetection::Surfaces SurfaceDetection::detect_surfaces_within(const Eigen::Affine3f &center,
                                                                    const Eigen::Vector3f &extents,
                                                                    const Surface::ProvideLevel provide_inliers,
                                                                    const Surface::ProvideLevel provide_shape,
                                                                    const Surface::ProvideLevel provide_mesh,
                                                                    const SurfaceVisualizationController &v) {
    duration<double> t_total, t_wait, t_pending_points, t_surfaces_within, t_expand_skipped, t_expand_existing,
            t_detect, t_expand_new, t_merge, t_return;
    auto t_total_start = high_resolution_clock::now();

    float roll, pitch, yaw;
    pcl::getEulerAngles(center, roll, pitch, yaw);
    ROS_DEBUG_STREAM("Detecting surfaces within " << extents.transpose() << " of " << center.translation().transpose()
                                                  << ", orientation " << roll << ", " << pitch << ", " << yaw);

    // Make the objects to store working data
    PointCloud all_pts;
    std::vector<int> considered_indices;
    std::vector<int> bb_indices;

    std::set<uint32_t> deleted_surfaces;
    std::set<uint32_t> collected_surfaces;
    std::set<uint32_t> updated_surfaces;
    std::set<std::pair<uint32_t, uint32_t>> merge_candidates;

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

    auto t_expand_skipped_start = high_resolution_clock::now();
    if (!expansion_skipped_.empty()) {
        for (auto expand_it = expansion_skipped_.begin(); expand_it != expansion_skipped_.end(); /* ++ is inside */) {
            auto expand_it_at_top = expand_it; // For debug
            if (collected_surfaces.find(*expand_it) != collected_surfaces.end()) {
                // Note: ALWAYS include it in updated_surfaces, even if expansion didn't find any points, because
                // from the point of view of the client it is new
                updated_surfaces.insert(*expand_it);

                // Erase and return reference to the next element -- this takes the place of ++
                expand_it = expansion_skipped_.erase(expand_it);

                if (considered_indices.empty()) {
                    // If there aren't any points, there's no point in doing anything, but logically expansion has been
                    // run, so it's vaid to erase this surface from expansion_skipped_
                    continue;
                }

                auto &surface = get_surface(*expand_it);
                auto new_ind = expand_surfaces_.expandAlongPlane(all_pts, surface.boundary(), surface.model(),
                                                                 surface.pose_float(), new_labels, merge_candidates,
                                                                 *expand_it, v);

                assert(std::count_if(new_labels.begin(), new_labels.end(),
                                     [&](const int label) { return label == surface.id(); }) == new_ind.size() &&
                       "expandAlongPlane's modification to new_labels does not agree with the list of "
                       "indices that it returned");

                if (new_ind.empty()) {
                    continue;
                }

                surface.add_inliers(all_pts, new_ind);
                // Was already added to updated_surfaces before
                ROS_DEBUG_STREAM(surface.print_color() << "Expanded existing surface " << *expand_it << " by "
                                                       << new_ind.size() << " points");

            } else {
                ++expand_it;
                ROS_DEBUG_STREAM("Surface " << *expand_it << " needs expansion but is not in this query box");
            }
            assert(expand_it != expand_it_at_top && "Got to bottom of loop withing changing expand_it");
        }
    } else {
        ROS_DEBUG_STREAM("No skipped surfaces to expand");
    }
    t_expand_skipped = high_resolution_clock::now() - t_expand_skipped_start;

    if (considered_indices.size() < 5) {
        ROS_DEBUG_STREAM("Skipping everything else because " << considered_indices.size()
                                                             << " points is not enough to do anything");
        // If they're not processed, it's not valid to delete them
        considered_indices.clear();
        // Need to initialize the times or else Bad Things happen
        t_expand_existing = t_detect = t_expand_new = duration<double>(0);
    } else {
        // Expand existing surfaces, if needed
        // Note that it is intentional that there are two expansion phases, because the first removes points that then
        // do not need to be considered by detect surfaces, which happens in between.
        auto t_expand_existing_start = high_resolution_clock::now();
        std::size_t n_indices_removed = 0;
        if (optimistic_) {
            ROS_DEBUG_STREAM("Optimistic mode does not require expanding existing surfaces");
        } else if (collected_surfaces.empty()) {
            ROS_DEBUG_STREAM("Not doing expansion because there are no surfaces");
        } else {
            // Pessimistic mode: only expand as far as the (padded) query box
            // This call removes the points it adds to a surface from considered_indices
            for (const uint32_t surface_id : collected_surfaces) {
                auto &surface = get_surface(surface_id);
                auto new_ind = expand_surfaces_.expandAlongPlane(all_pts, surface.boundary(), surface.model(),
                                                                 surface.pose_float(), new_labels, merge_candidates,
                                                                 surface_id, v);
                if (new_ind.empty()) {
                    continue;
                }

                surface.add_inliers(all_pts, new_ind);
                ROS_DEBUG_STREAM(surface.print_color() << "Expanded existing surface " << surface_id << " by "
                                                       << new_ind.size() << " points");

                updated_surfaces.insert(surface_id);
                n_indices_removed += new_ind.size();
            }
            ROS_DEBUG_STREAM("Expand surfaces reduced number of indices by "
                             << n_indices_removed << " to " << considered_indices.size() - n_indices_removed);
        }
        t_expand_existing = high_resolution_clock::now() - t_expand_existing_start;

        if (considered_indices.size() - n_indices_removed < min_pts_in_surface_) {
            ROS_DEBUG_STREAM("Not doing detection on " << considered_indices.size() << " points");
            t_detect = t_expand_new = duration<double>(0);
        } else {
            considered_indices.erase(std::remove_if(considered_indices.begin(), considered_indices.end(),
                                                    [&](const uint32_t idx) { return new_labels[idx] >= 0; }),
                                     considered_indices.end());

            // Detect new surfaces (updates new_labels to add labels for new surfaces)
            auto t_detect_start = high_resolution_clock::now();
            std::vector<uint32_t> new_surfaces;
            detect_surfaces_.detect_surfaces(all_pts, considered_indices, new_labels, v, [&](Surface new_surface) {
                collected_surfaces.insert(new_surface.id());
                updated_surfaces.insert(new_surface.id());
                new_surfaces.push_back(new_surface.id());
                add_surface(std::move(new_surface));
            });
            t_detect = high_resolution_clock::now() - t_detect_start;

            // Update new surfaces
            auto t_expand_new_start = high_resolution_clock::now();
            for (const auto new_surface_id : new_surfaces) {
                auto &surface = get_surface(new_surface_id);
                auto new_ind = expand_surfaces_.expandAlongPlane(all_pts, surface.boundary_or_inliers(),
                                                                 surface.model(), surface.pose_float(), new_labels,
                                                                 merge_candidates, new_surface_id, v);
                if (new_ind.empty()) {
                    continue;
                }

                surface.add_inliers(all_pts, new_ind);
                ROS_DEBUG_STREAM(surface.print_color() << "Expanded new surface " << new_surface_id << " by "
                                                       << new_ind.size() << " points");
            }
            t_expand_new = high_resolution_clock::now() - t_expand_new_start;
        }
    }

    ROS_DEBUG_STREAM("Found " << merge_candidates.size() << " merge candidates");
    std::vector<int> merged(surfaces_.rbegin()->first + 1, -1);

    auto t_merge_start = high_resolution_clock::now();
    auto max_angle = 2 * std::asin(2 * perpendicular_distance_ / min_width_);
    for (const auto pair : merge_candidates) {
        uint32_t first = pair.first, second = pair.second;
        // Follow the possibly-multiple-step path from the original candidate to whichever surface that surface now
        // belongs to.
        while (merged[first] != -1) {
            first = static_cast<uint32_t>(merged[first]);
        }
        while (merged[second] != -1) {
            second = static_cast<uint32_t>(merged[second]);
        }

        if (first == second) {
            ROS_DEBUG_STREAM("No merge between surface " << first << " and itself");
            continue;
        }

        ROS_DEBUG_STREAM("Considering merge between " << first << " and " << second << " (originally " << pair.first
                                                      << " and " << pair.second << ")");

        if ((pair.first != first || pair.second != second) &&
            merge_candidates.find(std::minmax(first, second)) != merge_candidates.end()) {
            // Don't check this pair twice (if it's in the set it either has been or will be checked)
            ROS_DEBUG_STREAM("No merge because this pair was already in the list");
            continue;
        }

        Surface &first_surface = get_surface(first);
        Surface &second_surface = get_surface(second);

        if (!first_surface.has_plane()) {
            first_surface.update_plane(build_surface_.compute_plane(first_surface, v));
        }
        if (!second_surface.has_plane()) {
            second_surface.update_plane(build_surface_.compute_plane(second_surface, v));
        }

        if (pcl::getAngle3D(first_surface.normal(), second_surface.normal()) > max_angle) {
            ROS_DEBUG_STREAM("No merge because the angle between the normals is too large ("
                             << pcl::getAngle3D(first_surface.normal(), second_surface.normal()) << ")");
            continue;
        }

        auto in_order = (first_surface.size() < second_surface.size());
        Surface &smaller = in_order ? first_surface : second_surface;
        Surface &larger = in_order ? second_surface : first_surface;
        auto plane = Eigen::Vector4f(larger.model().values.data());

        int outliers_allowed = static_cast<int>(smaller.size() * 0.05);
        for (const auto pt : smaller.inliers()) {
            if (pcl::pointToPlaneDistance(pt, plane) > perpendicular_distance_) {
                outliers_allowed--;
                if (outliers_allowed <= 0) {
                    break;
                }
            }
        }

        if (outliers_allowed > 0) {
            // Then fewer than 5% of smaller was outside of the plane of larger, so do merge
            ROS_DEBUG_STREAM(smaller.print_color() << "Merging surface " << smaller.id() << larger.print_color()
                                                   << " into surface " << larger.id());
            larger.add_inliers(smaller.inliers());
            deleted_surfaces.insert(smaller.id());
            collected_surfaces.erase(smaller.id());
            updated_surfaces.erase(smaller.id());
            merged[smaller.id()] = larger.id();
            remove_surface(smaller);
        } else {
            ROS_DEBUG_STREAM("No merge because there would be too many outliers");
        }
    }
    t_merge = high_resolution_clock::now() - t_merge_start;

    // Make an object to populate
    Surfaces new_surfaces;
    new_surfaces.header.frame_id = target_frame_;

    auto t_return_start = high_resolution_clock::now();

    std::copy(deleted_surfaces.begin(), deleted_surfaces.end(), std::back_inserter(new_surfaces.deleted_surfaces));
    std::set_difference(collected_surfaces.begin(), collected_surfaces.end(), updated_surfaces.begin(),
                        updated_surfaces.end(), std::back_inserter(new_surfaces.unchanged_surfaces));

    for (auto &sid : updated_surfaces) {
        Surface &surface = get_surface(sid);

        // It's not possible for inliers to be out of date
        bool get_inliers = (provide_inliers != Surface::NEVER);
        // It is possible for the others to be out of date
        bool get_shape =
            (provide_shape == Surface::ALWAYS || (provide_shape == Surface::IF_AVAILABLE && surface.has_shape()));
        bool get_mesh =
            (provide_mesh == Surface::ALWAYS || (provide_mesh == Surface::IF_AVAILABLE && surface.has_mesh()));

        // Make sure all necessary data is actually avalible
        // TODO If multiple need computing, compute them together, because it can share data that way
        if (!surface.has_plane()) {
            ROS_WARN("Once refactoring is done, I think surfaces should always have an updated plane at the end of "
                     "detect_surfaces?");
            ROS_DEBUG_STREAM(surface.print_color() << "Updating surface " << surface.id()
                                                   << "'s plane before returning");
            surface.update_plane(build_surface_.compute_plane(surface, v));
        }
        if (get_shape && !surface.has_shape()) {
            ROS_DEBUG_STREAM(surface.print_color() << "Updating surface " << surface.id()
                                                   << "'s shape before returning");
            surface.update_shape(build_surface_.compute_shape(surface, v));
        }
        if (get_mesh && !surface.has_mesh()) {
            ROS_DEBUG_STREAM(surface.print_color() << "Updating surface " << surface.id()
                                                   << "'s mesh before returning");
            surface.update_mesh(build_surface_.compute_mesh(surface));
        }

        ROS_DEBUG_STREAM(surface.print_color() << "Returning surface " << surface.id());
        new_surfaces.surfaces.push_back(surface.get_data(get_inliers, get_shape, get_mesh));
    }
    t_return = high_resolution_clock::now() - t_return_start;

    t_total = high_resolution_clock::now() - t_total_start;
    duration<double> t_other = t_total - (t_wait + t_pending_points + t_surfaces_within + t_expand_skipped +
                                          t_expand_existing + t_detect + t_expand_new + t_merge + t_return);
    ROS_DEBUG_STREAM("Surface detection took " << duration_cast<milliseconds>(t_total).count()
                                               << "ms. Time breakdown:");
    TIME_FMT(t_wait, t_total, "waiting for previous run's cleanup");
    TIME_FMT(t_pending_points, t_total, "getting pending points");
    TIME_FMT(t_surfaces_within, t_total, "getting existing surfaces");
    TIME_FMT(t_expand_skipped, t_total, "running expansion on skipped surfaces");
    TIME_FMT(t_expand_existing, t_total, "running expansion on existing surfaces");
    TIME_FMT(t_detect, t_total, "detecting new surfaces");
    TIME_FMT(t_expand_new, t_total, "running expansion on new surfaces");
    TIME_FMT(t_merge, t_total, "evaluating merge candidates");
    TIME_FMT(t_return, t_total, "building return object");
    TIME_FMT(t_other, t_total, "other");

    cleanup_done_ =
        std::async(std::launch::async,
                   std::bind(&SurfaceDetection::cleanup, this, std::move(all_pts), std::move(updated_surfaces),
                             std::move(deleted_surfaces), std::move(bb_indices), std::move(new_labels), std::cref(v)));

    return new_surfaces;
}

void SurfaceDetection::cleanup(const PointCloud &processed_pts, const std::set<uint32_t> &updated,
                               const std::set<uint32_t> &deleted, std::vector<int> rm_indices,
                               const std::vector<int> labels, const SurfaceVisualizationController &v) {
    ROS_DEBUG_STREAM_NAMED("cleanup", "Cleaning up " << updated.size() << " updated surfaces and " << deleted.size()
                                                     << " deleted ones");

    if (!processed_pts.empty()) {
        auto n_rm_indices_before = rm_indices.size();
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
        ROS_DEBUG_STREAM_NAMED("cleanup", "Removed voxels of " << rm_indices.size()
                                                               << " new inliers and points inside query box ("
                                                               << n_rm_indices_before << " from the query box)");
    }

    // If no surfaces updated or deleted, there's no more work to do
    if (updated.empty() && deleted.empty()) return;

    update_collector_surfaces(v);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Updated surfaces octree");

    publish_updated_surfaces(updated, v);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Updated markers of updated surfaces");
    publish_deleted_surfaces(deleted, v);
    ROS_DEBUG_STREAM_NAMED("cleanup", "Deleted markers of deleted surfaces");
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

const Surface &SurfaceDetection::get_surface(uint32_t surface_id) const {
    auto sit = surfaces_.find(surface_id);
    assert(sit != surfaces_.end() && "Tried to get a surface we don't have");
    return sit->second;
}

Surface &SurfaceDetection::get_surface(uint32_t surface_id) {
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
    const uint32_t remove_id = surface.id();
    auto n_removed = surfaces_.erase(remove_id);
    assert(n_removed > 0 && "Tried to remove a surface that didn't exist");
    assert(n_removed == 1 && "Tried to remove one surface and accidentally removed multiple, somehow");
    //    collect_points_.remove_surface(remove_id);
}

void SurfaceDetection::publish_deleted_surfaces(const std::set<uint32_t> &ids,
                                                const SurfaceVisualizationController &v) const {
    for (auto id : ids) {
        v.remove_marker("pose", id);
        v.remove_marker("normal", id);
        v.remove_marker("pose_x", id);
        v.remove_marker("pose_y", id);
        v.remove_marker("polygons", id);
        v.remove_marker("mesh", id);
        ROS_DEBUG_STREAM("Removed markers for " << id);
    }
}

void SurfaceDetection::publish_updated_surfaces(const std::set<uint32_t> &ids,
                                                const SurfaceVisualizationController &v) const {
    for (auto id : ids) {
        const auto &surface = get_surface(id);
//        if (surface.has_plane()) {
//            v.pose("pose", surface);
//            v.plane_normal("normal", surface);
//        }
//        if (surface.has_shape()) {
//            v.polygons("polygons", surface);
//        }
//        if (surface.has_mesh()) {
            v.mesh("mesh", surface);
//        }
    }
}

void SurfaceDetection::update_collector_surfaces(const SurfaceVisualizationController &v) {
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
    int highest_label = -1;
    for (auto sit : surfaces_) {
        auto &surface = sit.second;

        if (surface.needs_cleanup()) {
            if (!surface.has_shape()) {
                surface.update_shape(build_surface_.compute_shape(surface, v));
            }
            surface.update_tiling(build_surface_.tile_surface(surface));
        }

        *cloud += surface.tiling();
        highest_label = std::max(highest_label, static_cast<int>(surface.id()));
    }

    ROS_DEBUG_STREAM("Updating surfaces with " << cloud->size() << " points from " << surfaces_.size() << " surfaces");
    collect_points_.update_surfaces(std::move(cloud), highest_label);
}

size_t SurfaceDetection::get_num_pending_points() { return collect_points_.num_pending_points(); }

void SurfaceDetection::add_surface(Surface surface) {
    auto result = surfaces_.emplace(surface.id(), std::move(surface));
    assert(result.second && "Tried to insert a duplicate surface");
}
}