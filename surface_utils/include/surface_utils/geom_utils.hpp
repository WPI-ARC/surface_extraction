//
// Created by will on 6/29/16.
//

#ifndef PROJECT_GEOM_UTILS_HPP
#define PROJECT_GEOM_UTILS_HPP

#include <surface_types/Surface.hpp>

namespace surface_geom_utils {
using namespace surface_types;

float point_sqr_dist(float x1, float y1, float x2, float y2) {
    float dx = x1 - x2;
    float dy = y1 - y2;
    return dx * dx + dy * dy;
}

template <typename PointCloud>
bool is_xy_in_tiling(const Surface &surface, const PointCloud &pts, float x, float y, double dist) {
    auto sqr_dist = dist * dist;
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
            if (point_sqr_dist(x, y, v.x, v.y) < sqr_dist || point_sqr_dist(x, y, w.x, w.y) < sqr_dist) {
                return false;
            }
            // The actual check
            const auto t = ((x - v.x) * (w.x - v.x) + (y - v.y) * (w.y - v.y)) / point_sqr_dist(v.x, v.y, w.x, w.y);
            // point was inside the segment (if outside, it was either caught by the first case or is OK)
            if (0 < t && t < 1) {
                // proj is the closest point on the segment to the query point
                const auto proj_x = v.x + t * (w.x - v.x);
                const auto proj_y = v.y + t * (w.y - v.y);
                if (point_sqr_dist(x, y, proj_x, proj_y) < sqr_dist) {
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
}

#endif // PROJECT_GEOM_UTILS_HPP
