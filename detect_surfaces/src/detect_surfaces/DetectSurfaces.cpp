//
// Created by will on 6/21/16.
//

#include "detect_surfaces/DetectSurfaces.h"

// PCL
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/time.h>

#include <surface_types/SurfaceData.hpp>

// Utils
#include <surface_utils/pcl_utils.hpp>
#include <surface_utils/SurfaceVisualizationController.hpp>
#include <surface_utils/color_generator.hpp>
#include <surface_utils/smart_ptr.hpp>
#include <arc_utilities/pretty_print.hpp>

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

void DetectSurfaces::detect_surfaces(const PointCloud &cloud, const std::vector<int> &indices,
                                     std::vector<int> &new_labels, const SurfaceVisualizationController &v,
                                     std::function<void(const Surface &)> callback) {
    std::vector<int> filtered_indices = radius_filter(cloud, indices);
    ROS_DEBUG_STREAM("Radius filter reduced " << indices.size() << " indices to " << filtered_indices.size());

    if (filtered_indices.size() < min_points_per_surface_) {
        return;
    }

    NormalCloud::Ptr normals = get_normals(cloud, filtered_indices);
    ROS_ERROR_STREAM_COND(filtered_indices.size() != normals->size(), "Expected normals to have "
                                                                          << filtered_indices.size()
                                                                          << " points, but it had " << normals->size());
    assert(filtered_indices.size() == normals->size() && "Normals cloud has the wrong number of points");

    assert(cloud.sensor_origin_.isApprox(normals->sensor_origin_) && "Get normals did not preserve sensor origin");
    assert(cloud.sensor_orientation_.isApprox(normals->sensor_orientation_) &&
           "Get normals did not preserve sensor orientation");

    v.points<Normal>("normals", normals);
    v.normal_vectors("normal_vectors", normals);

    // Build a search object that can be shared among the algorithms
    NormalSearch::Ptr normal_search = boost::make_shared<pcl::search::KdTree<Normal>>();
    normal_search->setInputCloud(normals);

    int n_regions = 0, n_planes = 0, n_euclidean = 0, n_filtered = 0;

    // Welcome to callback hell!
    region_segmentation(normals, normal_search, [&, this](Indices region) {
        n_regions++;
        sac_segmentation_and_fit(normals, normal_search, region, [&, this](Indices inliers, Model model) {
            n_planes++;
            euclidean_segmentation(normals, normal_search, inliers, [&, this](Indices segment) {
                n_euclidean++;
                find_transform_and_filter(normals, segment, [&, this](Eigen::Affine3f transform) {
                    n_filtered++;

                    // segment refers to normals, which is a subset of cloud, so it needs to be reindexed to
                    // refer to cloud before being returned
                    auto inlier_indices = surfaces_pcl_utils::reindex(filtered_indices, segment.indices);
                    auto surface = Surface({cloud, inlier_indices}, model, transform.cast<double>());

                    v.plane_normal("new_surface_normal", surface);

                    // Update new_labels (note new_labels[i] refers to cloud[i])
                    for (auto &idx : inlier_indices) {
                        new_labels[idx] = static_cast<int>(surface.id());
                    }

                    callback(surface);
                });
            });
        });
    });

    ROS_DEBUG_STREAM("DetectSurfaces found " << n_regions << " regions, " << n_planes << " planes, " << n_euclidean
                                             << " euclidean segements, " << n_filtered << " surfaces after filtering");
}

std::vector<int> DetectSurfaces::radius_filter(const PointCloud &cloud, const std::vector<int> &indices) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setRadiusSearch(mls_radius_);
    filter.setMinNeighborsInRadius(3);

    filter.setInputCloud(boost_fake_shared(cloud));
    filter.setIndices(boost_fake_shared(indices));

    std::vector<int> output;

    filter.filter(output);
    return output;
}

DetectSurfaces::NormalCloud::Ptr DetectSurfaces::get_normals(const PointCloud &cloud, const std::vector<int> &indices) {
    pcl::MovingLeastSquares<Point, Normal> mls;
    mls.setSearchRadius(mls_radius_);
    mls.setComputeNormals(true);
    mls.setPolynomialFit(false);
    mls.setPolynomialOrder(0);

    mls.setInputCloud(boost_fake_shared(cloud));
    mls.setIndices(boost_fake_shared(indices));

    // Do the reconstruction
    auto normals = boost::make_shared<NormalCloud>();
    mls.process(*normals);

    ROS_ERROR_STREAM_COND(normals->size() != indices.size(),
                          "Expected normals to have " << indices.size() << " points but it had " << normals->size());
    assert(normals->size() <= indices.size() && "Normals size is greater than indices size");
    assert(normals->size() >= indices.size() && "Normals size is less than indices size");

    normals->sensor_origin_ = cloud.sensor_origin_;
    normals->sensor_orientation_ = cloud.sensor_orientation_;

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

    //    ROS_INFO_STREAM("Region segmentation found " << clusters.size() << " regions");

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
            // Make sure the normal is oriented towards the cloud's acquisition viewpoint
            normals->sensor_origin_[3] = 1; // Required to ensure the correctness of the `if` statement
            // The correctness of the `if` statement depends on the first 3 values of the model being normalized
            assert(std::abs(Eigen::Map<Eigen::Vector3f>(model.values.data(), 3).norm() - 1) < 1e-4 &&
                   "Coefficients aren't normalized");
            if (Eigen::Map<Eigen::Vector4f>(model.values.data(), 4).dot(normals->sensor_origin_) < 0) {
                ROS_DEBUG_STREAM("Orienting plane normal towards point <"
                                 << normals->sensor_origin_[0] << ", " << normals->sensor_origin_[1] << ", "
                                 << normals->sensor_origin_[2] << ", " << normals->sensor_origin_[3] << ">");
                model.values[0] *= -1;
                model.values[1] *= -1;
                model.values[2] *= -1;
                model.values[3] *= -1;
            }
            // Make sure I didn't mess it up
            assert(std::abs(Eigen::Map<Eigen::Vector3f>(model.values.data(), 3).norm() - 1) < 1e-4 &&
                   "Coefficients aren't normalized after re-orienting");
            assert(Eigen::Map<Eigen::Vector4f>(model.values.data(), 4).dot(normals->sensor_origin_) >= 0 &&
                   "Model isn't oriented towards viewpoint after re-orienting");

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

        // Copies everything in remaining_src EXCEPT the contents of inliers into remaining_temp
        std::set_difference(remaining_indices->indices.begin(), remaining_indices->indices.end(),
                            inliers.indices.begin(), inliers.indices.end(),
                            std::back_inserter(remaining_indices_tmp.indices));
        remaining_indices->indices.swap(remaining_indices_tmp.indices);
    }

    //    ROS_INFO_STREAM("SAC segmentation found " << n_found_clusters << " planes");

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
    //    ROS_INFO_STREAM("Euclidean segmentation found " << clusters.size() << " clusters");

    std::for_each(clusters.begin(), clusters.end(), callback);
}

void DetectSurfaces::find_transform_and_filter(NormalCloud::Ptr &normals, pcl::PointIndices &inliers,
                                               std::function<void(Eigen::Affine3f)> callback) {
    // Lifted from pcl::getPrincipalTransformation
    // (not using that because I want to be able to pass indices)
    Eigen::Affine3f tf;
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    Eigen::Vector4f centroid;

    pcl::computeMeanAndCovarianceMatrix(*normals, inliers, covariance_matrix, centroid);

    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vects;
    Eigen::Vector3f eigen_vals;
    pcl::eigen33(covariance_matrix, eigen_vects, eigen_vals);

    tf.translation() = centroid.head(3);
    tf.linear() = eigen_vects;

    // This puts x along the major axis instead of z
    tf.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()));

    auto tf_inverse = tf.inverse();

    if (true) { // TODO: Configuration parameter to enable width check
        // Note: Don't start at zero, because if there aren't points around zero that incorrectly increases the spread
        float max_y = std::numeric_limits<float>::min(), min_y = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::min(), min_z = std::numeric_limits<float>::max();
        for (const int point_idx : inliers.indices) {
            Normal pt = pcl::transformPoint((*normals)[point_idx], tf_inverse);

            max_y = std::max(max_y, pt.y);
            min_y = std::min(min_y, pt.y);
            max_z = std::max(max_z, pt.z);
            min_z = std::min(min_z, pt.z);
        }

        assert((max_z - min_z) * perpendicular_distance_ &&
               "Z-spread was not less than twice the perpendicular distance");

        if (max_y - min_y < min_plane_width_) {
            return;
        }
    }

    // Make sure the orientation of the transform puts the sensor origin in +z direction
    Eigen::Vector3f origin_tf = tf * normals->sensor_origin_.head<3>();
    ROS_DEBUG_STREAM("Sensor origin relative to plane: " << origin_tf.transpose());
    if (origin_tf[2] < 0) {
        ROS_DEBUG_STREAM("Flippin' transform");
        // Rotate by pi around the y axis to flip the transform
        tf.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
    }

    callback(tf);
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
