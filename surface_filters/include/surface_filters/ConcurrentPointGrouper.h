//
// Created by will on 4/9/16.
//

#ifndef SURFACE_FILTERS_ATOMICPOINTGROUP_H
#define SURFACE_FILTERS_ATOMICPOINTGROUP_H

// Standard and boost includes
#include <mutex>

// PCL inclues
#include <pcl/point_cloud.h>

namespace surface_filters {

template <typename PointT>
class ConcurrentPointGrouper {
    // Typedefs
    using PointType = PointT; // Expose PointType publicly
    using PointCloudType = typename pcl::PointCloud<PointT>;

private: // Data members
    typename PointCloudType::Ptr cloud_ptr_;
    std::recursive_mutex mutex_;

public:
    ConcurrentPointGrouper() : cloud_ptr_(nullptr), mutex_() {}

    void add(const typename PointCloudType::ConstPtr &new_cloud, const pcl::PointIndices::ConstPtr &new_indices) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (new_indices) {
            add(new_cloud, new_indices->indices);
        } else {
            add(new_cloud);
        }
    }

    void add(const typename PointCloudType::ConstPtr &new_cloud, const std::vector<int> &new_indices) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        // Requirement: The cloud can't be null
        assert(new_cloud);
        // Requirement: If this already has some points, the new points must have the same frame
        assert(!cloud_ptr_ || cloud_ptr_->header.frame_id == new_cloud->header.frame_id);

        if (cloud_ptr_) {
            // PointCloud doesn't have a utility method for this that avoids
            // unnecessary work
            cloud_ptr_->reserve(cloud_ptr_->size() + new_indices.size());
            for (const auto &index : new_indices) {
                cloud_ptr_->points.push_back((*new_cloud)[index]);
            }
            // Important! Reestablish internal PointCloud invariants
            cloud_ptr_->width = static_cast<uint32_t>(cloud_ptr_->points.size());
            cloud_ptr_->height = 1;
        } else {
            cloud_ptr_ = boost::make_shared<PointCloudType>(*new_cloud, new_indices);
        }

        cloud_ptr_->header = new_cloud->header;
        cloud_ptr_->sensor_orientation_ = new_cloud->sensor_orientation_;
        cloud_ptr_->sensor_origin_ = new_cloud->sensor_origin_;
    }

    void add(const typename PointCloudType::ConstPtr &new_cloud) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        // Requirement: The cloud can't be null
        assert(new_cloud);
        // Requirement: If this already has some points, the new points must have the same frame
        assert(!cloud_ptr_ || cloud_ptr_->header.frame_id == new_cloud->header.frame_id);

        if (cloud_ptr_) {
            *cloud_ptr_ += *new_cloud;
        } else {
            cloud_ptr_ = boost::make_shared<PointCloudType>(*new_cloud);
        }

        cloud_ptr_->header = new_cloud->header;
        cloud_ptr_->sensor_orientation_ = new_cloud->sensor_orientation_;
        cloud_ptr_->sensor_origin_ = new_cloud->sensor_origin_;
    }

    typename PointCloudType::Ptr finish_group() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        return std::move(cloud_ptr_); // NOTE use of std::move leaves cloud_ptr_ set to nullptr
    }

    template <typename Predicate>
    typename PointCloudType::Ptr finish_group_if(Predicate pred) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (cloud_ptr_ && pred(cloud_ptr_)) {
            return std::move(cloud_ptr_); // NOTE use of std::move leaves cloud_ptr_ set to nullptr
        } else {
            return nullptr;
        }
    }

    typename std::pair<typename PointCloudType::ConstPtr, pcl::PointIndices::ConstPtr>
    finish_group_with(const typename PointCloudType::ConstPtr &new_cloud,
                      const pcl::PointIndices::ConstPtr &new_indices = nullptr) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);

        if (cloud_ptr_) {
            add(new_cloud, new_indices);
            return {std::move(cloud_ptr_), nullptr}; // NOTE use of std::move leaves cloud_ptr_ set to nullptr
        } else {
            return {new_cloud, new_indices};
        }
    }

    std::size_t size() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return cloud_ptr_ ? cloud_ptr_->size() : 0;
    }
};
}

#endif // SURFACE_FILTERS_ATOMICPOINTGROUP_H
