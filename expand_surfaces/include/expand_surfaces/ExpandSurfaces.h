//
// Created by will on 6/20/16.
//

#ifndef PROJECT_EXPANDSURFACES_HPP
#define PROJECT_EXPANDSURFACES_HPP

#include <utility>
#include <map>
#include <functional>
#include <pcl/point_cloud.h>

// Forward declarations
namespace pcl {
    struct PointXYZ;
    class PointIndices;
    class ModelCoefficients;
    namespace search {template <typename PointT> class Search;}
}

namespace surface_types {
    class Surface;
}

class ExpandSurfaces {
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> PointCloud;
    typedef surface_types::Surface Surface;

    typedef pcl::search::Search<Point> Search;

    typedef std::pair<PointCloud, pcl::PointIndices> CloudIndexPair;

public:
    ExpandSurfaces(double perpendicular_dist, double parallel_dist);

    pcl::PointIndices expand_surfaces(const std::map<int, Surface> &surfaces, const CloudIndexPair &input,
                                      std::function<void(Surface)> callback);

private:
    pcl::PointIndices filterWithinRadiusConnected(const Search &search, const PointCloud::Ptr &edge_points,
                                                  const pcl::PointIndices &remaining_indices) const;

    pcl::PointIndices filterWithinModelDistance(const PointCloud::ConstPtr &input, const pcl::PointIndices &indices,
                                                const pcl::ModelCoefficients &coeff);

protected:
    double perpendicular_distance_;
    double parallel_distance_;
};

#endif // PROJECT_EXPANDSURFACES_HPP
