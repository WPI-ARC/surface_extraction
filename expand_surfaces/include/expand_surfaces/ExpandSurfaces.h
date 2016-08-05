//
// Created by will on 6/20/16.
//

#ifndef PROJECT_EXPANDSURFACES_HPP
#define PROJECT_EXPANDSURFACES_HPP

#include <utility>
#include <map>
#include <functional>
#include <pcl/point_cloud.h>
#include <set>
#include <surface_utils/SurfaceVisualizationController.hpp>

// Forward declarations
namespace pcl {
struct PointXYZ;
class ModelCoefficients;
namespace search {
template <typename PointT>
class Search;
}
}

class Surface;

class ExpandSurfaces {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;

    typedef pcl::search::Search<Point> Search;

public:
    ExpandSurfaces(double perpendicular_dist, double parallel_dist, double disc);

    std::vector<int>
    expandAlongPlane(const ExpandSurfaces::PointCloud &cloud,
                         const ExpandSurfaces::PointCloud &edge_points,
                         const pcl::ModelCoefficients &model, const Eigen::Affine3f &tf,
                         std::vector<int> &processed,
                         std::set<std::pair<uint32_t, uint32_t>> &merge_candidates,
                         const uint32_t label, const SurfaceVisualizationController &v) const;

protected:
    double perpendicular_distance_;
    double parallel_distance_;
    double discretization_;
};

#endif // PROJECT_EXPANDSURFACES_HPP
