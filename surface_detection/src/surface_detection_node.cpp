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
#include "surface_utils/eigen_helpers_conversions.hpp"
#include "surface_detection/SurfaceDetection.h"
#include "surface_utils/SurfaceVisualizationController.hpp"
#include <arc_utilities/pretty_print.hpp>

using surface_msgs2::SurfaceDetectionRequest;
using surface_msgs2::SurfaceDetectionResponse;

using EigenHelpersConversions::GeometryPoseToEigenAffine3f;
using EigenHelpersConversions::GeometryVector3ToEigenVector3f;

using surface_detection::SurfaceDetection;

class get_surfaces {
public:
    SurfaceDetection &surface_detection;
    SurfaceVisualizationController vis;
    ros::Subscriber &points_sub;

    get_surfaces(SurfaceDetection &sd, ros::NodeHandle *nhp, std::string frame, ros::Subscriber &s)
        : surface_detection(sd), vis{nhp, frame}, points_sub(s) {}

    bool go(SurfaceDetectionRequest &req, SurfaceDetectionResponse &resp) {
        ROS_INFO_STREAM("Recieved request for surfaces");

        // Disable collecting new points during the entire planning process
        // Ideally this would re-enable it after planning is done; for now I do that by restarting the entire system
        points_sub.shutdown();

        auto center = GeometryPoseToEigenAffine3f(req.center);
        auto extents = GeometryVector3ToEigenVector3f(req.extents);

        vis.bounding_box(center, extents);

        ros::WallTime compute_start = ros::WallTime::now();
        resp = surface_detection.detect_surfaces_within(center, extents, Surface::ProvideLevel(req.provide_inliers),
                                                        Surface::ProvideLevel(req.provide_shape),
                                                        Surface::ProvideLevel(req.provide_mesh), vis);
        ros::WallDuration elapsed_time = ros::WallTime::now() - compute_start;
        // Can't send a WallDuration in a message
        resp.computation_time = ros::Duration(elapsed_time.sec, elapsed_time.nsec);
        ROS_INFO_STREAM("Replying with " << resp.surfaces.size() << " new/updated surfaces, "
                                         << resp.unchanged_surfaces.size() << " unchanged, and "
                                         << resp.deleted_surfaces.size() << " ones after " << std::fixed
                                         << std::setprecision(3) << resp.computation_time.toSec() << "s");

        return true;
    }
};

int main(int argc, char **argv) {
    // Setup
    ros::init(argc, argv, "surface_detection");
    ros::NodeHandle n;       // This one is for subscribing and publishing (public)
    ros::NodeHandle pn("~"); // This one is for getting private parameters

    const std::string target_frame = pn.param("target_frame", std::string("/world"));

    // We want the alpha shape corresponding to a disc of diameter = parallel_distance.
    // CGAL defines alpha as the square of the radius of the disc (different to the original definition in the paper)
    auto parallel_distance = pn.param("parallel_distance", 0.06);
    auto alpha = std::pow(parallel_distance / 2.0, 2);

    SurfaceDetection surface_detection(
        pn.param("discretization", 0.02), pn.param("perpendicular_distance", 0.04), parallel_distance,
        pn.param("point_inside_threshold", 0.1), pn.param("mls_radius", 0.12), pn.param("min_points_per_surface", 50),
        pn.param("min_plane_width", 0.15), alpha, pn.param("extrusion_distance", 0.02f),
        pn.param("optimistic", true), target_frame, pn.param("camera_frame", std::string("/left_camera_frame")));

    ROS_INFO_STREAM("Connected to ROS");

    // Make publishers
    auto pp_pub = n.advertise<SurfaceDetection::PointCloud>(
        pn.param("unprocessed_points_topic", std::string("/pending_points")), 1);
    auto sp_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/surface_points", 1);

    // Make subscribers
    auto sub = n.subscribe(pn.param("input_points_topic", std::string("/scan_cloud")), 100,
                           &SurfaceDetection::add_points, &surface_detection);

    // Make timers
    auto pp_timer =
        n.createTimer(ros::Duration(1), [&surface_detection, &pp_pub, &sp_pub, &target_frame](const ros::TimerEvent &) {
            if (pp_pub.getNumSubscribers() == 0) {
                return;
            }

            auto cloud = surface_detection.get_pending_points();
            cloud.header.frame_id = target_frame;

            ROS_DEBUG_STREAM_DELAYED_THROTTLE(10, "Currently have " << cloud.size() << " pending points");
            ROS_DEBUG_STREAM_ONCE("Publishing first pending-points cloud of size " << cloud.size());

            pp_pub.publish(cloud);

            auto surf = surface_detection.get_surface_points();
            surf.header.frame_id = target_frame;
            sp_pub.publish(surf);
        });

    get_surfaces getter(surface_detection, pn.param("enable_visualization", true) ? &n : nullptr, target_frame, sub);

    float start_surface;
    if (pn.getParam("initial_surface", start_surface)) {
        surface_detection.add_start_surface(start_surface, start_surface, getter.vis);
        ROS_DEBUG_STREAM("Added start surface with dimensions " << start_surface << "x" << start_surface);
    } else {
        ROS_DEBUG_STREAM("No start surface to add");
    }

    int n_points_to_wait_for = 0;
    if (pn.getParam("wait_for_points", n_points_to_wait_for)) {
        ros::Rate r(2);
        while (ros::ok() &&
               surface_detection.get_num_pending_points() < static_cast<std::size_t>(n_points_to_wait_for)) {
            ROS_INFO_STREAM_DELAYED_THROTTLE(2, "Waiting for " << n_points_to_wait_for
                                                               << " points before advertising service");
            r.sleep();
            ros::spinOnce();
        }
    } else {
        ROS_INFO_STREAM("Not waiting for points");
    }

    auto cloud = surface_detection.get_pending_points();
    cloud.header.frame_id = target_frame;
    pp_pub.publish(cloud);
    pp_timer.stop();

    // Make service provider
    auto srv = n.advertiseService("get_surfaces", &get_surfaces::go, &getter);

    ROS_INFO_STREAM("Setup complete!");
    ros::spin();

    return 0;
}