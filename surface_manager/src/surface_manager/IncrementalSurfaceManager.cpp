//
// Created by will on 2/15/16.
//

#include <pluginlib/class_list_macros.h>
#include <surface_manager/IncrementalSurfaceManager.h>

void surface_manager::IncrementalSurfaceManager::onInit() {
    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();

    // Subscribe to the input directly b/c there is nothing to synchronize
    sub_input_ = private_nh.subscribe<PointCloudIn>(
        "input", 10, bind(&IncrementalSurfaceManager::synchronized_input_callback, this, _1));

    service_ = private_nh.advertiseService("get_surfaces", &IncrementalSurfaceManager::get_surfaces, this);

    NODELET_DEBUG("[%s::onInit] IncrementalSurfaceManager Nodelet successfully created with connections:\n"
                  " - [subscriber] input       : %s\n",
                  getName().c_str(), getMTPrivateNodeHandle().resolveName("input").c_str());
}

void surface_manager::IncrementalSurfaceManager::synchronized_input_callback(PointCloudIn::ConstPtr &input) const {
    grouper_.add(input, changed(downsample(input)));
}

auto surface_manager::IncrementalSurfaceManager::downsample(PointCloudIn::ConstPtr &input) const -> PointCloudIn::Ptr& {
    auto output = boost::make_shared<PointCloudIn>();

    pcl::VoxelGrid<PointIn> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(resolution_, resolution_, resolution_);
    sor.filter(*output);

    return output;
}

auto surface_manager::IncrementalSurfaceManager::changed(PointCloudIn::ConstPtr &input) const -> pcl::PointIndicesPtr& {
    change_detect_.setInputCloud(input);
    change_detect_.addPointsFromInputCloud();

    pcl::PointIndicesPtr indices = boost::make_shared<pcl::PointIndices>();

    change_detect_.getPointIndicesFromNewVoxels(indices->indices, 0);

    // Swap in preparation for the next call
    change_detect_.switchBuffers();

    return indices;
}

bool surface_manager::IncrementalSurfaceManager::get_surfaces(SurfaceDetectionRequest &req,
                                                              SurfaceDetectionResponse &resp) {
    auto points = grouper_.extract_subset(bind(&IncrementalSurfaceManager::crop, this, _1, req));



    return false;
}

auto surface_manager::IncrementalSurfaceManager::crop(PointCloudIn::ConstPtr &input, SurfaceDetectionRequest &req) const -> std::pair<PointCloudIn::Ptr, PointCloudIn::Ptr> {
    auto box_points = boost::make_shared<PointCloudIn>();
    pcl::CropBox<PointIn> cb(true);

    cb.setTransform(EigenHelpersConversions::GeometryPoseToEigenAffine3f(req.center));
    cb.setMax({static_cast<float>(req.extents.x), static_cast<float>(req.extents.y),
               static_cast<float>(req.extents.z), 1});
    cb.setMin({-static_cast<float>(req.extents.x), -static_cast<float>(req.extents.y),
               -static_cast<float>(req.extents.z), 1});
    cb.setInputCloud(input);
    cb.filter(*box_points);

    return std::pair<PointCloudIn::Ptr, PointCloudIn::Ptr>(
            box_points, boost::make_shared<PointCloudIn>(*input, *cb.getRemovedIndices()));
}

//auto surface_manager::IncrementalSurfaceManager::mls(PointCloudIn::ConstPtr &input) const -> PointCloudNormal::Ptr & {
//    auto normals = boost::make_shared<PointCloudNormal>();
//
//    pcl::MovingLeastSquares<PointIn, PointNormal> mls;
//    mls.setSearchRadius(0.15);
//    mls.setPolynomialFit(false);
//    mls.setComputeNormals(true);
//
//    mls.process(*normals);
//
//    return normals;
//}
//
//auto surface_manager::IncrementalSurfaceManager::region_growing(PointCloudIn::ConstPtr &input,
//                                                                PointCloudNormal::ConstPtr &normals) const -> std::vector<pcl::PointIndices> {
//    auto clusters = std::vector<pcl::PointIndices>();
//
//    pcl::RegionGrowing<PointIn, PointNormal> rgs;
//    rgs.setMinClusterSize(500);
//    rgs.setSmoothModeFlag(false);
//    rgs.setCurvatureThreshold(0.03);
//
//    rgs.extract(clusters);
//
//    return clusters;
//}
//
//auto surface_manager::IncrementalSurfaceManager::ransac(PointCloudIn::ConstPtr &input,
//                                                        pcl::PointIndices &cluster) const -> std::vector<std::pair<pcl::PointIndices, pcl::ModelCoefficients>> {
//
//}


PLUGINLIB_EXPORT_CLASS(surface_manager::IncrementalSurfaceManager, nodelet::Nodelet)
