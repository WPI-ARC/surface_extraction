//
// Created by will on 4/12/16.
//

#ifndef SURFACE_FILTERS_FILTERSURFACEINLIERS_H
#define SURFACE_FILTERS_FILTERSURFACEINLIERS_H


#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_hull.h>

#include <surfaces/Surface.hpp>
#include <surfaces/Surfaces.hpp>
#include <surfaces/utils.hpp>

namespace surface_filters {

template <typename PointT>
class FilterSurfaceInliers {
    using Point = PointT;
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointIndices = pcl::PointIndices;
    using Surface = typename surfaces::Surface<PointT>;
    using Surfaces = typename surfaces::Surfaces<PointT>;

private:
    double distance_ = 0.;
    message_filters::Subscriber<Surfaces> sub_surfaces_;
    boost::shared_ptr<message_filters::Cache<Surfaces>> surfaces_cache_;

public:
    void subscribe(ros::NodeHandle pnh, std::string name, int max_queue_size) {
        sub_surfaces_.subscribe(pnh, name, max_queue_size);
        surfaces_cache_ = boost::make_shared<message_filters::Cache<Surfaces>>(sub_surfaces_, 1);
    }

    void setDistance(double distance) { distance_ = distance; }
    double getDistance() { return distance_; }

    pcl::IndicesPtr getFilteredIndices(const typename Surfaces::ConstPtr &surfaces,
                                       const typename PointCloud::ConstPtr &cloud_in,
                                       const PointIndices::ConstPtr &indices = nullptr);

    pcl::IndicesPtr getFilteredIndices(const typename PointCloud::ConstPtr &cloud_in,
                                       const PointIndices::ConstPtr &indices = nullptr);

    pcl::IndicesPtr filterWithinModelDistance(const typename PointCloud::ConstPtr &input,
                                              const pcl::IndicesConstPtr &indices, const pcl::ModelCoefficients &coeff);

    pcl::IndicesPtr filterWithinHull(const typename PointCloud::ConstPtr &input, const pcl::IndicesConstPtr &indices,
                                     const typename PointCloud::Ptr &hull_cloud,
                                     const std::vector<pcl::Vertices> &hull_polygons);
};

}


template <typename PointT>
pcl::IndicesPtr
surface_filters::FilterSurfaceInliers<PointT>::getFilteredIndices(const typename Surfaces::ConstPtr &surfaces,
                                                                  const typename PointCloud::ConstPtr &cloud_in,
                                                                  const PointIndices::ConstPtr &indices) {
    // pcl::ScopeTime("Filtering indices");

    pcl::IndicesPtr indices_remaining = indices ? boost::make_shared<std::vector<int>>(indices->indices)
                                                : surfaces::all_indices<PointT>(cloud_in);
//    auto original_size = indices_remaining->size();

    std::sort(indices_remaining->begin(), indices_remaining->end());
    for (const Surface &surface : surfaces->surfaces) {
        // Clip point cloud to plane
        pcl::IndicesPtr indices_inplane = this->filterWithinModelDistance(cloud_in, indices_remaining, surface.model);
//        ROS_DEBUG_STREAM("Found " << indices_inplane->size() << " points in the plane of surface " << surface.id);
        if (indices_inplane->size() == 0) continue;

        // Clip point cloud to be within hull
        auto hull_cloud = boost::make_shared<PointCloud>();
        pcl::fromPCLPointCloud2(surface.concave_hull.cloud, *hull_cloud);
        pcl::IndicesPtr indices_inhull = this->filterWithinHull(cloud_in, indices_inplane, hull_cloud,
                                                                surface.concave_hull.polygons);
//        ROS_DEBUG_STREAM("Found " << indices_inhull->size() << " points in the hull of surface " << surface.id);
        if (indices_inhull->size() == 0) continue;

        std::sort(indices_inhull->begin(), indices_inhull->end());

        pcl::IndicesPtr indices_remaining_tmp = boost::make_shared<std::vector<int>>();
        std::set_difference(indices_remaining->begin(), indices_remaining->end(), indices_inhull->begin(),
                            indices_inhull->end(), std::back_inserter(*indices_remaining_tmp));

        indices_remaining.swap(indices_remaining_tmp);
//        ROS_DEBUG_STREAM((original_size - indices_remaining->size()) << " points removed after surface " << surface.id);
    }

    return indices_remaining;
}

template <typename PointT>
pcl::IndicesPtr
surface_filters::FilterSurfaceInliers<PointT>::getFilteredIndices(const typename PointCloud::ConstPtr &cloud_in,
                                                                  const PointIndices::ConstPtr &indices) {
    if (!surfaces_cache_) return nullptr;
    auto surfaces = surfaces_cache_->getElemBeforeTime(ros::Time::now());
    return surfaces ? getFilteredIndices(surfaces, cloud_in, indices) : nullptr;
}

template <typename PointT>
pcl::IndicesPtr
surface_filters::FilterSurfaceInliers<PointT>::filterWithinModelDistance(const typename PointCloud::ConstPtr &input,
                                                                         const pcl::IndicesConstPtr &indices,
                                                                         const pcl::ModelCoefficients &coeff) {
    // pcl::ScopeTime("Filter within model distance");
    assert(distance_ > 0);

    auto model = pcl::SampleConsensusModelPlane<Point>(input, *indices);

    pcl::IndicesPtr output_indices = boost::make_shared<std::vector<int>>();
    model.selectWithinDistance(Eigen::Vector4f(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]),
                               distance_, *output_indices);

    return output_indices;
}

template <typename PointT>
pcl::IndicesPtr surface_filters::FilterSurfaceInliers<PointT>::filterWithinHull(
        const typename PointCloud::ConstPtr &input, const pcl::IndicesConstPtr &indices,
        const typename PointCloud::Ptr &hull_cloud, const std::vector<pcl::Vertices> &hull_polygons) {
    pcl::CropHull<Point> crophull;
    crophull.setDim(2);
    crophull.setHullCloud(hull_cloud);
    crophull.setHullIndices(hull_polygons);
    crophull.setInputCloud(input);
    crophull.setIndices(indices);

    pcl::IndicesPtr output_indices = boost::make_shared<std::vector<int>>();
    crophull.filter(*output_indices);

    return output_indices;
}


#endif // SURFACE_FILTERS_FILTERSURFACEINLIERS_H
