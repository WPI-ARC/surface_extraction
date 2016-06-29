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
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <surface_utils/smart_ptr.hpp>

namespace surface_detection {
class SurfaceDetection {
public:
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef surface_types::Surface Surface;
    typedef surface_types::Surfaces Surfaces;

    SurfaceDetection(double discretization, double perpendicular_dist, double parallel_dist, double mls_radius,
                     unsigned int min_pts_in_surface, double min_plane_width, double alpha, float extrusion_distance,
                     std::string target_frame, std::string camera_frame)
        : target_frame_(target_frame), parallel_distance_(parallel_dist), perpendicular_distance_(perpendicular_dist),
          sqr_perpendicular_distance_(perpendicular_dist * perpendicular_dist),
          // State
          surfaces_(),
          // Implementation
          collect_points_(discretization, perpendicular_dist, target_frame, camera_frame),
          expand_surfaces_(perpendicular_dist, parallel_dist),
          detect_surfaces_(perpendicular_dist, parallel_dist, mls_radius, min_pts_in_surface, min_plane_width),
          build_surface_(perpendicular_dist, alpha, extrusion_distance) {}

    void add_points(const PointCloud::ConstPtr &points) { collect_points_.add_points(points); }

    PointCloud get_pending_points() { return collect_points_.get_pending_points(); }

    Surfaces detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents, ProgressListener p) {
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
            new_surfaces.surfaces.push_back(surfaces_[surface_id]);
        });
        ROS_DEBUG_STREAM("Started with " << new_surfaces.surfaces.size() << " existing surfaces");

        input.second = expand_surfaces_.expand_surfaces(new_surfaces.surfaces, input, [&, this](Surface s_expanded) {
            build_surface_.build_updated_surface(s_expanded, p, [&, this](Surface new_s) {
                ROS_DEBUG_STREAM("Updated surface with id " << new_s.id);

                this->add_or_update_surface(new_s, p);
                auto posn = std::find_if(new_surfaces.surfaces.begin(), new_surfaces.surfaces.end(),
                                         [&new_s](const Surface &s) { return s.id == new_s.id; });
                assert(posn != new_surfaces.surfaces.end() && "Surface disappeared from new_surfaces during expansion");
                (*posn) = new_s;
            });
        });

        // Build a search object for the input points
        pcl::search::KdTree<Point> search(false);
        search.setInputCloud(boost::shared_ptr<PointCloud>(&input.first, null_deleter()),
                             boost::shared_ptr<std::vector<int>>(&input.second.indices, null_deleter()));

        // Make some shorter names for the sake of formatting
        using Indices = pcl::PointIndices;
        using Model = pcl::ModelCoefficients;

        // Detect new surfaces
        detect_surfaces_.detect_surfaces(input, p, [&](Indices indices, Model model, Eigen::Affine3f tf) {
            expand_surfaces_.expand_new_surface(input.first, search, indices, tf, [&, this](pcl::PointIndices in) {
                PointCloud new_inliers_cloud(input.first, in.indices);

                build_surface_.build_new_surface(new_inliers_cloud, model, tf.cast<double>(), p, [&, this](Surface s) {
                    ROS_DEBUG_STREAM("Got a new surface with id " << s.id);

                    this->add_or_update_surface(s, p);
                    new_surfaces.surfaces.push_back(s);
                });
            });
        });
        ROS_DEBUG_STREAM("Detected a total of " << new_surfaces.surfaces.size() << " surfaces");

        return new_surfaces;
    }

    Surfaces detect_surfaces_within(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        return detect_surfaces_within(center, extents, ProgressListener());
    }

    PointCloud generate_tiling(const Surface &surface) {
        PointCloud cloud;
        PointCloud boundary_flat;
        pcl::transformPointCloud(surface.inliers, boundary_flat, surface.pose.inverse());

        assert(std::all_of(boundary_flat.begin(), boundary_flat.end(), [](Point p) { return p.z < .5 && p.z > -.5; }));

        Point min, max;
        pcl::getMinMax3D(boundary_flat, min, max);

        double discretization = parallel_distance_ / 4.; // arbitrary
        auto n = 0;
        // Offset by discretization / 2 to avoid testing the minimum, which will by definition be outside
        for (float y = min.y + discretization / 2.; y < max.y; y += discretization) {
            // X is the widest axis so iterate x most rapidly (not sure if that makes any difference w/ this algorithm)
            for (float x = min.x + discretization / 2.; x < max.x; x += discretization) {
                // For each (x, y) pair, skip if it's outside, or if it's within perpendicular_distance_ of any edge
                if (is_xy_in_tiling(surface, boundary_flat, x, y)) {
                    cloud.push_back(pcl::transformPoint(Point(x, y, 0), surface.pose.cast<float>()));
                }
                n++;
            }
        }

        ROS_DEBUG_STREAM("Tiled " << float(cloud.size()) / float(n) * 100 << "% of the bounding box");

        return cloud;
    }

    float point_sqr_dist(float x1, float y1, float x2, float y2) const {
        float dx = x1 - x2;
        float dy = y1 - y2;
        return dx * dx + dy * dy;
    }

    bool is_xy_in_tiling(const Surface &surface, const PointCloud &pts, float x, float y) const {
        // For each (x, y) pair, skip if it's outside, or if it's within perpendicular_distance_ of any edge
        bool is_inside = false;
        // PiP algorithm: count segments that cross the ray from the test point along the +x axis
        // Inside if there is an odd number, outside if there is an even number
        for (auto &polygon : surface.polygons) {
            auto prev_vertex = polygon.vertices.back();
            for (auto &vertex : polygon.vertices) {
                if (prev_vertex == vertex) continue; // This way it handles explicitly and implicitly closed polygons

                const auto v = pts[prev_vertex], w = pts[vertex];
                // If segment is too close to the line, skip it
                // First, the easy case: within the given distance of the endpoints
                if (point_sqr_dist(x, y, v.x, v.y) < sqr_perpendicular_distance_ ||
                    point_sqr_dist(x, y, w.x, w.y) < sqr_perpendicular_distance_) {
                    return false;
                }
                // The actual check
                const auto t = ((x - v.x) * (w.x - v.x) + (y - v.y) * (w.y - v.y)) / point_sqr_dist(v.x, v.y, w.x, w.y);
                // point was inside the segment (if outside, it was either caught by the first case or is OK)
                if (0 < t && t < 1) {
                    // proj is the closest point on the segment to the query point
                    const auto proj_x = v.x + t * (w.x - v.x);
                    const auto proj_y = v.y + t * (w.y - v.y);
                    if (point_sqr_dist(x, y, proj_x, proj_y) < sqr_perpendicular_distance_) {
                        return false;
                    }
                }

                // Point-in-polygon -- code from
                // http://bbs.dartmouth.edu/~fangq/MATH/download/source/Determining%20if%20a%20point%20lies%20on%20the%20interior%20of%20a%20polygon.htm
                if ((((v.y <= y) && (y < w.y)) || ((w.y <= y) && (y < v.y))) &&
                    (x < (w.x - v.x) * (y - v.y) / (w.y - v.y) + v.x)) {
                    is_inside = !is_inside;
                }

                prev_vertex = vertex;
            }
        }

        return is_inside;
    }

    void add_or_update_surface(Surface &updated_surface, ProgressListener &p) {
        surfaces_[updated_surface.id] = updated_surface;
        auto tiling = generate_tiling(updated_surface);
        p.points<Point>("tiling", tiling.makeShared());
        collect_points_.add_surface(tiling, updated_surface.id);
    }

private:
    // Configuration
    std::string target_frame_;
    double parallel_distance_;
    double perpendicular_distance_;
    double sqr_perpendicular_distance_;

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
