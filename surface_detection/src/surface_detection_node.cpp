//
// Created by will on 6/18/16.
//

#include <ros/ros.h>
// pcl_ros/point_cloud.h enables subscribing to topics using PCL types
#include <pcl_ros/point_cloud.h>
#include <surface_msgs2/SurfaceDetection.h>
#include <surface_msgs2/SurfaceDetectionRequest.h>
#include <surface_msgs2/SurfaceDetectionResponse.h>

#include "surface_detection/SurfaceDetection.h"
#include <arc_utilities/eigen_helpers_conversions.hpp>

using Request = surface_msgs2::SurfaceDetectionRequest;
using Response = surface_msgs2::SurfaceDetectionResponse;

using EigenHelpersConversions::GeometryPoseToEigenAffine3f;
using EigenHelpersConversions::GeometryVector3ToEigenVector3f;

using surface_detection::SurfaceDetection;

// Parameters
// TODO: Get these from parameter server
std::string frame = "/world";
const double discretization = 0.1;
const double perpendicular_distance = 0.025;

int main(int argc, char **argv) {
    // Setup
    ros::init(argc, argv, "surface_detection");
    ros::NodeHandle n;
    SurfaceDetection surface_detection(discretization, perpendicular_distance);

    ROS_DEBUG_STREAM("Connected to ROS");

    // Make publishers
    auto pp_pub = n.advertise<SurfaceDetection::PointCloud>("pending_points", 1);

    // Make subscribers
    auto sub = n.subscribe("scan_cloud", 1000, &SurfaceDetection::add_points, &surface_detection);

    // Make timers
    auto pp_timer = n.createTimer(ros::Duration(1), [&surface_detection, &pp_pub](const ros::TimerEvent &) {
        auto cloud = surface_detection.get_pending_points();
        cloud.header.frame_id = frame;
        pp_pub.publish(cloud);
    });

    // I SURE DO LOVE MAKING FUNCTORS ALL OVER HTE GODDAMN PLACE
    class get_surfaces {
        SurfaceDetection &surface_detection;
    public:

        get_surfaces(SurfaceDetection &sd) : surface_detection(sd) {}

        bool go (Request &req, Response &resp) {
            std::tie(resp.surfaces, resp.surface_meshes) = surface_detection.detect_surfaces_within(
                    GeometryPoseToEigenAffine3f(req.center), GeometryVector3ToEigenVector3f(req.extents));
            return true;
        }
    };
    get_surfaces getter(surface_detection);
    // Make service provider
    auto srv = n.advertiseService("get_surfaces", &get_surfaces::go, &getter);

    ros::spin();

    return 0;
}