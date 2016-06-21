//
// Created by will on 6/20/16.
//

#ifndef PROJECT_EXPANDSURFACES_HPP
#define PROJECT_EXPANDSURFACES_HPP

#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <surface_types/Surfaces.hpp>
#include <surface_utils/smart_ptr.hpp>

template <typename PointT>
class ExpandSurfaces {
    typedef surface_types::Surface<PointT> Surface;
    typedef surface_types::Surfaces<PointT> Surfaces;
    typedef surface_types::SurfaceMesh SurfaceMesh;
    typedef surface_types::SurfaceMeshes SurfaceMeshes;

    typedef std::pair<Surface, SurfaceMesh> SurfaceMeshPair;
    typedef std::pair<Surfaces, SurfaceMeshes> SurfaceMeshListPair;

    typedef pcl::search::Search<PointT> Search;

    typedef pcl::PointCloud<PointT> PointCloud;

    typedef std::pair<PointCloud, pcl::PointIndices> CloudIndexPair;

public:
    ExpandSurfaces(double perpendicular_dist, double parallel_dist)
        : perpendicular_distance_(perpendicular_dist), parallel_distance_(parallel_dist) {
        assert(perpendicular_distance_ > 0);
        assert(parallel_distance_ > 0);
    }

    pcl::PointIndices filterWithinRadiusConnected(const Search &search, const typename PointCloud::Ptr &edge_points,
                                                  const pcl::PointIndices &remaining_indices) const {
        std::set<int> within_radius_indices;

        std::queue<PointT> to_search(std::deque<PointT>(edge_points->begin(), edge_points->end()));

        while (!to_search.empty()) {
            const auto &point = to_search.front();

            std::vector<int> tmp_indices;
            std::vector<float> tmp_sqrdistances; // Only needed to fill a parameter

            search.radiusSearch(point, parallel_distance_, tmp_indices, tmp_sqrdistances);

            // TODO: Unify this with filterWithinHull by filtering tmp_indices here instead of later
            // Should speed this up because it won't explore too far from the surface in the perpendicular direction

            for (const auto &nearby_index : tmp_indices) {
                const auto insert_result = within_radius_indices.insert(nearby_index);

                if (insert_result.second) { // If the index didn't already exist in the list
                    to_search.push(search.getInputCloud()->at(static_cast<std::size_t>(nearby_index)));
                }
            }

            to_search.pop();
        }

        pcl::PointIndices within_radius_nodupes;
        // This may reserve more space than necessary, but it shouldn't be much more.
        within_radius_nodupes.indices.reserve(within_radius_indices.size());

        // Sortedness Invariants:
        // within_radius_indices is sorted because it's a set
        // remaining_indices' sortedness is a loop invariant
        std::set_intersection(within_radius_indices.begin(), within_radius_indices.end(),
                              remaining_indices.indices.begin(), remaining_indices.indices.end(),
                              std::back_inserter(within_radius_nodupes.indices));

        return within_radius_nodupes;
    }

    pcl::PointIndices filterWithinModelDistance(const typename PointCloud::ConstPtr &input, const pcl::PointIndices &indices,
                                                const pcl::ModelCoefficients &coeff) {
        // pcl::ScopeTime("Filter within model distance");

        auto model = pcl::SampleConsensusModelPlane<PointT>(input, indices.indices);

        pcl::PointIndices output_indices;
        model.selectWithinDistance(Eigen::Vector4f(coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]),
                                   perpendicular_distance_, output_indices.indices);

        return output_indices;
    }

    template <typename CallbackT>
    pcl::PointIndices expand_surfaces(const std::map<int, SurfaceMeshPair> &surfaces, const CloudIndexPair &input,
                                      CallbackT callback) {
        auto cloud = boost::shared_ptr<const PointCloud>(&input.first, null_deleter());
        auto indices = boost::shared_ptr<const pcl::PointIndices>(&input.second, null_deleter());

        if (cloud->size() == 0 || indices->indices.size() == 0) {
            return input.second;
        }

        pcl::search::KdTree<PointT> search(false);
        search.setInputCloud(cloud, boost::shared_ptr<const std::vector<int>>(&input.second.indices, null_deleter()));

        pcl::PointIndices remaining_indices;
        pcl::PointIndices remaining_indices_tmp;
        remaining_indices.indices = indices->indices;

        for (const auto &old_surface_pair : surfaces) {
            const auto &old_surface = old_surface_pair.second.first;

            auto hull_cloud = boost::make_shared<pcl::PointCloud<PointT>>();
            pcl::fromPCLPointCloud2(old_surface.concave_hull.cloud, *hull_cloud);

            //
            // FILTER TO POINTS NEAR SURFACE BOUNDARY
            //
            auto radius_filtered = filterWithinRadiusConnected(search, hull_cloud, remaining_indices);
            if (radius_filtered.indices.size() == 0) continue;

            //
            // FILTER TO POINTS ON THE SURFACE
            //
            auto distance_filtered = filterWithinModelDistance(cloud, radius_filtered, old_surface.model);
            if (distance_filtered.indices.size() == 0) continue;

            //
            // ADD NEW POINTS TO SURFACE
            //
            surface_types::Surface<PointT> new_surface(old_surface);
            new_surface.concave_hull = pcl::PolygonMesh();
            new_surface.inliers += pcl::PointCloud<PointT>(*cloud, distance_filtered.indices);

            callback(new_surface);

            //
            // REMOVE THOSE POINTS FROM THE INPUT
            //
            remaining_indices_tmp.indices.reserve(remaining_indices.indices.size() - distance_filtered.indices.size());
            std::set_difference(remaining_indices.indices.begin(), remaining_indices.indices.end(),
                                distance_filtered.indices.begin(), distance_filtered.indices.end(),
                                std::back_inserter(remaining_indices_tmp.indices));
            remaining_indices.indices.swap(remaining_indices_tmp.indices);
        }

        // Always publish remaining indices
        return remaining_indices;
    }

protected:
    double perpendicular_distance_;
    double parallel_distance_;
};

#endif // PROJECT_EXPANDSURFACES_HPP
