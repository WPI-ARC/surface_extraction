//
// Created by will on 6/21/16.
//

#include "detect_surfaces/DetectSurfaces.h"

// PCL
#include <pcl/surface/mls.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

// Surfaces
#include <surface_types/Surfaces.hpp>
#include <surface_types/SurfaceMesh.hpp>

// Utils
#include <surface_utils/pcl_utils.hpp>
#include <surface_utils/ProgressListener.hpp>
#include <surface_utils/color_generator.hpp>
#include <surface_utils/smart_ptr.hpp>

// Some much-needed shorter names for the sake of code formatting
using Indices = pcl::PointIndices;
using Model = pcl::ModelCoefficients;

DetectSurfaces::DetectSurfaces(double perpendicular_dist, double parallel_dist, double mls_radius,
                               unsigned int min_points_per_surface, double min_plane_width)
    : perpendicular_distance_(perpendicular_dist), parallel_distance_(parallel_dist), mls_radius_(mls_radius),
      min_points_per_surface_(min_points_per_surface), min_plane_width_(min_plane_width) {
    assert(perpendicular_distance_ > 0);
    assert(parallel_distance_ > 0);
    assert(mls_radius > 0);
    assert(min_points_per_surface_ >= 3);
}

void DetectSurfaces::detect_surfaces(const CloudIndexPair &input, ProgressListener &p,
                                     std::function<void(Indices, Model, Eigen::Affine3f)> callback) {
    auto normals = get_normals(input);
    p.points<Normal>("normals", normals);
    p.normal_vectors("normal_vectors", normals);

    // Build a search object that can be shared among the algorithms
    NormalSearch::Ptr normal_search = boost::make_shared<pcl::search::KdTree<Normal>>();
    normal_search->setInputCloud(normals);

    std::vector<Indices> planes;

    // Welcome to callback hell!
    region_segmentation(normals, normal_search, [&, this](Indices region) {
        sac_segmentation_and_fit(normals, normal_search, region, [&, this](Indices inliers, Model model) {
            euclidean_segmentation(normals, normal_search, inliers, [&, this](Indices segment) {
                find_transform_and_filter(normals, segment, [&, this](Eigen::Affine3f transform) {
                    planes.push_back(segment);
                    callback(segment, model, transform);
                });
            });
        });
    });

    p.points<pcl::PointXYZRGB>("new_planes", make_segment_colored_cloud(normals, planes));
}

DetectSurfaces::NormalCloud::Ptr DetectSurfaces::get_normals(const CloudIndexPair &input) {
    pcl::MovingLeastSquares<Point, Normal> mls;
    mls.setSearchRadius(mls_radius_);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(false);
    mls.setPolynomialOrder(0);

    mls.setInputCloud(boost::shared_ptr<const PointCloud>(&input.first, null_deleter()));
    mls.setIndices(boost::shared_ptr<const std::vector<int>>(&input.second.indices, null_deleter()));

    // Do the reconstruction
    auto normals = boost::make_shared<NormalCloud>();
    mls.process(*normals);

    return normals;
}

std::size_t DetectSurfaces::region_segmentation(NormalCloud::Ptr &normals, NormalSearch::Ptr &search,
                                                std::function<void(pcl::PointIndices)> callback) {
    pcl::RegionGrowing<Normal, Normal> rgs;
    rgs.setMinClusterSize(min_points_per_surface_);
    rgs.setSmoothModeFlag(false);

    rgs.setInputCloud(normals);
    rgs.setInputNormals(normals);
    rgs.setSearchMethod(search);

    // Do the reconstruction
    std::vector<pcl::PointIndices> clusters;
    rgs.extract(clusters);

    ROS_INFO_STREAM("Region segmentation found " << clusters.size() << " regions");

    std::reverse(clusters.begin(), clusters.end()); // For debuggging

    std::for_each(clusters.begin(), clusters.end(), callback);

    return clusters.size();
}

pcl::PointIndices
DetectSurfaces::sac_segmentation_and_fit(NormalCloud::Ptr &normals, NormalSearch::Ptr &search,
                                         pcl::PointIndices &region,
                                         std::function<void(pcl::PointIndices, pcl::ModelCoefficients)> callback) {
    pcl::SACSegmentationFromNormals<Normal, Normal> sac;
    sac.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(perpendicular_distance_);
    sac.setNormalDistanceWeight(0.1);
    sac.setOptimizeCoefficients(true);

    sac.setInputCloud(normals);
    sac.setInputNormals(normals);
//    sac.setSamplesMaxDist(parallel_distance_, search);

    auto remaining_indices = boost::make_shared<pcl::PointIndices>(region);
    pcl::PointIndices remaining_indices_tmp;
    int n_allowed_fails = 3;
    int n_found_clusters = 0;

    while (n_allowed_fails > 0) {
        pcl::PointIndices inliers;
        pcl::ModelCoefficients model;
        // Limit SAC to only the points in this cluster
        sac.setIndices(remaining_indices);

        // Do the SAC segmentation
        sac.segment(inliers, model);

        if (inliers.indices.size() >= min_points_per_surface_) {
            n_found_clusters++;
            callback(inliers, model);
        } else {
            n_allowed_fails -= 1;
            continue;
        }

        if (remaining_indices->indices.size() - inliers.indices.size() < min_points_per_surface_) {
            break;
        }

        // Only sort if necessary
        if (remaining_indices_tmp.indices.empty()) {
            std::sort(remaining_indices->indices.begin(), remaining_indices->indices.end());
        } else {
            remaining_indices_tmp.indices.clear();
        }

        // The inliers have no guarantee of sorted-ness
        std::sort(inliers.indices.begin(), inliers.indices.end());

        // Copies everything in remaining_src EXCEPT the contents of inliners into remaining_temp
        std::set_difference(remaining_indices->indices.begin(), remaining_indices->indices.end(),
                            inliers.indices.begin(), inliers.indices.end(),
                            std::back_inserter(remaining_indices_tmp.indices));
        remaining_indices->indices.swap(remaining_indices_tmp.indices);
    }

    ROS_INFO_STREAM("SAC segmentation found " << n_found_clusters << " planes");

    return *remaining_indices;
}

void DetectSurfaces::euclidean_segmentation(NormalCloud::Ptr &normals, NormalSearch::Ptr &search,
                                            pcl::PointIndices &inliers,
                                            std::function<void(pcl::PointIndices)> callback) {
    pcl::EuclideanClusterExtraction<Normal> seg;
    seg.setMinClusterSize(min_points_per_surface_);
    seg.setClusterTolerance(parallel_distance_);

    seg.setInputCloud(normals);
    seg.setIndices(boost::shared_ptr<const std::vector<int>>(&inliers.indices, null_deleter()));
    seg.setSearchMethod(search);

    std::vector<pcl::PointIndices> clusters;
    seg.extract(clusters);
    ROS_INFO_STREAM("Euclidean segmentation found " << clusters.size() << " clusters");

    std::for_each(clusters.begin(), clusters.end(), callback);
}

void DetectSurfaces::find_transform_and_filter(NormalCloud::Ptr &normals, pcl::PointIndices &inliers,
                                               std::function<void(Eigen::Affine3f)> callback) {
    pcl::PCA<Normal> pca(true); // true -> don't compute coefficients

    pca.setInputCloud(normals);
    pca.setIndices(boost::shared_ptr<const std::vector<int>>(&inliers.indices, null_deleter()));

    Normal pt;
    float max_y = 0, min_y = 0;
    for (const int point_idx : inliers.indices) {
        pca.project(normals->points[point_idx], pt);

        max_y = std::max(max_y, pt.y);
        min_y = std::min(min_y, pt.y);

        if (max_y - min_y < min_plane_width_) {
            ROS_INFO_STREAM("PCA discarded a surface for being only " << (max_y - min_y) << " m wide");
            return;
        }
    }

    // If it didn't return yet, then the cloud is wide enough
    callback(Eigen::Affine3f(pca.getEigenVectors()) * Eigen::Translation3f(pca.getMean().head(3)));
}

auto DetectSurfaces::make_segment_colored_cloud(NormalCloud::Ptr &normals, std::vector<pcl::PointIndices> &segments)
    -> DetectSurfaces::ColoredPointCloud::Ptr {
    auto result = boost::make_shared<ColoredPointCloud>();

    random_colors::color_generator gen(0.6, 0.99);

    std::size_t result_i = 0;
    for (auto &segment : segments) {
        // resize + assign is more efficient than reserve + push_back because of PointCloud implementation details
        result->resize(result->size() + segment.indices.size());

        auto color_tuple = gen.rgb();
        // This point object is reused in the loop
        ColoredPoint pt(static_cast<uint8_t>(std::get<0>(color_tuple) * 255),
                        static_cast<uint8_t>(std::get<1>(color_tuple) * 255),
                        static_cast<uint8_t>(std::get<2>(color_tuple) * 255));

        for (auto &index : segment.indices) {
            pt.x = (*normals)[index].x;
            pt.y = (*normals)[index].y;
            pt.z = (*normals)[index].z;

            // Assigns a copy
            (*result)[result_i] = pt;
            result_i++;
        }
    }

    return result;
}
