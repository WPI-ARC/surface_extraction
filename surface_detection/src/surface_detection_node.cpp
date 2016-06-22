//
// Created by will on 6/18/16.
//

#include <ros/ros.h>
// pcl_ros/point_cloud.h enables subscribing to topics using PCL types
#include <pcl_ros/point_cloud.h>
#include <surface_msgs2/SurfaceDetection.h>
#include <surface_msgs2/SurfaceDetectionRequest.h>
#include <surface_msgs2/SurfaceDetectionResponse.h>

#include <arc_utilities/eigen_helpers_conversions.hpp>
#include "surface_detection/SurfaceDetection.h"
#include "surface_utils/ProgressListener.hpp"
#include <arc_utilities/pretty_print.hpp>

using surface_msgs2::SurfaceDetectionRequest;
using surface_msgs2::SurfaceDetectionResponse;

using EigenHelpersConversions::GeometryPoseToEigenAffine3f;
using EigenHelpersConversions::GeometryVector3ToEigenVector3f;

using surface_detection::SurfaceDetection;

// Parameters
// TODO: Get these from parameter server
std::string frame = "/world";
const double discretization = 0.02;
const double perpendicular_distance = 0.025;
const double parallel_distance = 0.05;
const double mls_radius = 0.10;
const unsigned int min_points_per_surface = 50;
const double min_plane_width = 0.1;

int main(int argc, char **argv) {
    // Setup
    ros::init(argc, argv, "surface_detection");
    ros::NodeHandle n;
    SurfaceDetection surface_detection(discretization, perpendicular_distance, parallel_distance, mls_radius, min_points_per_surface, min_plane_width);

    ROS_INFO_STREAM("Connected to ROS");

    // Make publishers
    auto pp_pub = n.advertise<SurfaceDetection::PointCloud>("pending_points", 1);

    // Make subscribers
    auto sub = n.subscribe("scan_cloud", 1000, &SurfaceDetection::add_points, &surface_detection);

    // Make timers
    auto pp_timer = n.createTimer(ros::Duration(1), [&surface_detection, &pp_pub](const ros::TimerEvent &) {
        auto cloud = surface_detection.get_pending_points();
        cloud.header.frame_id = frame;

        ROS_DEBUG_STREAM_THROTTLE(10, "Currently have " << cloud.size() << " pending points");

        pp_pub.publish(cloud);
    });

    // I SURE DO LOVE MAKING FUNCTORS ALL OVER HTE GODDAMN PLACE
    class get_surfaces {
        SurfaceDetection &surface_detection;
        ProgressListener progress;
    public:

        get_surfaces(SurfaceDetection &sd, ProgressListener pl) : surface_detection(sd),  progress(pl) {}

        bool go (SurfaceDetectionRequest &req, SurfaceDetectionResponse &resp) {
            auto center = GeometryPoseToEigenAffine3f(req.center);
            auto extents = GeometryVector3ToEigenVector3f(req.extents);
            ROS_DEBUG_STREAM("Center: " << PrettyPrint::PrettyPrint(center) << ", Extents: <x: " << extents[0] << ", y: " << extents[1] << " z, " << extents[2] << ">");
            std::tie(resp.surfaces, resp.surface_meshes) = surface_detection.detect_surfaces_within(
                    center, extents, progress);
            return true;
        }
    };
    get_surfaces getter(surface_detection, ProgressListener(&n, frame));
    // Make service provider
    auto srv = n.advertiseService("get_surfaces", &get_surfaces::go, &getter);

    ROS_INFO_STREAM("Setup complete!");
    ros::spin();

    return 0;
}