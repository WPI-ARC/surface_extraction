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

    void expand_surfaces(const std::vector<Surface> &surfaces, const PointCloud &cloud, std::vector<int> &indices,
                             std::function<void(Surface)> callback);

    void expand_surface(const PointCloud &points, const pcl::search::Search<Point> &search,
                        const Surface &input_surface, std::function<void(std::vector<int>)> callback);

    std::vector<int> expandAlongPlane(const ExpandSurfaces::PointCloud &cloud,
                                      const ExpandSurfaces::Search &search,
                                      const ExpandSurfaces::PointCloud &edge_points,
                                      const Eigen::Affine3f &tf, std::vector<int> &processed,
                                      const uint32_t label) const;

private:
    std::vector<int> expandAlongPlane(const PointCloud &cloud, const Search &search,
                                      const PointCloud &edge_points, const Eigen::Affine3f &tf) const;

protected:
    double perpendicular_distance_;
    double parallel_distance_;
    double discretization_;
};

#endif // PROJECT_EXPANDSURFACES_HPP
