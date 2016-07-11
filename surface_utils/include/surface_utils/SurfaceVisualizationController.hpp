//
// Created by will on 6/21/16.
//

#ifndef PROJECT_PROGRESSLISTENER_HPP
#define PROJECT_PROGRESSLISTENER_HPP

// System includes
#include <thread>
#include <Eigen/Geometry>

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// PCL Includes
// pcl_ros/point_cloud.h enables publishing topics using PCL types
#include <pcl_ros/point_cloud.h>

// Local includes
#include <surface_types/Surface.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)

class SurfaceVisualizationController {
    typedef std::map<std::string, ros::Publisher> PubMap;
    typedef std::shared_ptr<ros::NodeHandle *> NHPtr;

public:
    // Use this constructor to disable SurfaceVisualizationController
    SurfaceVisualizationController() : nh_ptr_(nullptr), frame_(""), pubs_(nullptr) {}

    // Use this constructor to use SurfaceVisualizationController
    SurfaceVisualizationController(ros::NodeHandle *np, std::string frame)
        : nh_ptr_(std::make_shared<ros::NodeHandle *>(np)), frame_(frame), pubs_(std::make_shared<PubMap>()) {}

    // No copies allowed
    SurfaceVisualizationController(const SurfaceVisualizationController& other) = delete;
    SurfaceVisualizationController& operator=(SurfaceVisualizationController const&) = delete;

    template <typename Message, typename Func>
    void with_publisher(std::string name, Func function) const {
        // Note nh_ptr is a smart pointer to a raw pointer; the smart pointer always exist but check the inner pointer
        if (!*nh_ptr_) return;

        std::map<std::string, ros::Publisher>::iterator pub_it = pubs_->find(name);

        if (pub_it == pubs_->end()) {
            pub_it = pubs_->insert(std::make_pair(name, (*nh_ptr_)->advertise<Message>(name, 10))).first;
        }

        // Publishers are internally reference counted, making a copy here and on the next line is fine
        ros::Publisher pub = pub_it->second;

        // Lambda needs to be thread-safe (capture everything by value or by shared ptr which is captured by value)
        std::thread([name, function, pub]() {
            ros::Rate r(2); // 2Hz -> 0.5 seconds apart
            ros::Time stop_time = ros::Time::now() + ros::Duration(10);
            while (pub.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                r.sleep();
            }

            function(pub);

        }).detach();
    }

    template <typename Func>
    void with_marker_publisher(Func function) const {
        with_publisher<visualization_msgs::Marker>("surface_markers", function);
    }

    template <typename PointT>
    void points(std::string name, const typename pcl::PointCloud<PointT>::Ptr &points) const {
        with_publisher<pcl::PointCloud<PointT>>(name, [=](const ros::Publisher &pub) {
            points->header.frame_id = frame_;
            pub.publish(points);
        });
    }

    template <typename PointT>
    void pair(std::string name, typename std::pair<pcl::PointCloud<PointT>, pcl::PointIndices> pair) const {
        if (!nh_ptr_) return;

        auto points = boost::make_shared<pcl::PointCloud<PointT>>(pair.first, pair.second.indices);
        this->points<PointT>(name, points);
    }

    void normal_vectors(std::string name, pcl::PointCloud<pcl::PointNormal>::Ptr &normals_cloud) const {
        with_publisher<geometry_msgs::PoseArray>(name, [=](const ros::Publisher &pub) {
            geometry_msgs::PoseArray poses;
            poses.header.frame_id = frame_;

            Eigen::Affine3d ztf(Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitY()));

            for (auto &pt : *normals_cloud) {
                geometry_msgs::Pose pose;
                geometry_msgs::Quaternion msg;

                // extracting surface normals
                tf::Vector3 axis_vector(pt.normal[0], pt.normal[1], pt.normal[2]);
                tf::Vector3 up_vector(0.0, 0.0, 1.0);

                tf::Vector3 right_vector = axis_vector.cross(up_vector);
                right_vector.normalized();
                tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
                q.normalize();
                tf::quaternionTFToMsg(q, msg);

                // adding pose to pose array
                pose.position.x = pt.x;
                pose.position.y = pt.y;
                pose.position.z = pt.z;
                pose.orientation = msg;

                // PoseArrays always have the arrow on the x axis, but I want them on the z axis, so
                // transform each pose
                // This definitely duplicates some work, but whatever
                geometry_msgs::Pose result;
                Eigen::Affine3d tf;
                tf::poseMsgToEigen(pose, tf);
                tf::poseEigenToMsg(tf * ztf, result);

                poses.poses.push_back(result);
            }

            pub.publish(poses);
        });
    }

    void vector(std::string name, Eigen::Vector3d direction, Eigen::Vector3d origin) const {
        geometry_msgs::Pose p;
        tf::pointEigenToMsg(origin, p.position);
        // Rviz's pose display puts an arrow on the x axis, so use the x axis to compute quaternion
        const auto q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction);
        tf::quaternionEigenToMsg(q, p.orientation);
        pose(name, p);
    }

    void pose(std::string name, const geometry_msgs::PoseStamped &p) const {
        with_publisher<geometry_msgs::PoseStamped>(name, [=](const ros::Publisher &pub) { pub.publish(p); });
    }

    void pose(std::string name, const geometry_msgs::Pose &p) const {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = frame_;
        ps.pose = p;
        this->pose(name, ps);
    }

    void pose(std::string name, const Eigen::Affine3d &e) const {
        geometry_msgs::Pose p;
        tf::poseEigenToMsg(e, p);
        this->pose(name, p);
    }

    void pose(std::string name, const Eigen::Affine3f &e) const {
        geometry_msgs::Pose p;
        tf::poseEigenToMsg(e.cast<double>(), p);
        this->pose(name, p);
    }

    void poses(std::string name, std::vector<geometry_msgs::Pose> &p) const {
        with_publisher<geometry_msgs::PoseArray>(name, [=](const ros::Publisher &pub) {
            geometry_msgs::PoseArray ps;
            ps.header.frame_id = frame_;
            //            ps.poses = p;
            ps.poses.reserve(p.size());

            // PoseArrays always have the arrow on the x axis, but I want them on the z axis, so transform each pose
            Eigen::Affine3d ztf(Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitY()));
            std::transform(p.begin(), p.end(), std::back_inserter(ps.poses), [&ztf](const geometry_msgs::Pose &pose) {
                geometry_msgs::Pose result;
                Eigen::Affine3d tf;
                tf::poseMsgToEigen(pose, tf);
                tf::poseEigenToMsg(tf * ztf, result);
                return result;
            });
            pub.publish(ps);
        });
    }

    void poses(std::string name, std::vector<Eigen::Affine3d> &e) const {
        std::vector<geometry_msgs::Pose> ps;
        ps.resize(e.size());
        for (std::size_t i = 0; i < e.size(); i++)
            tf::poseEigenToMsg(e[i], ps[i]);
        this->poses(name, ps);
    }

    void poses(std::string name, std::vector<Eigen::Affine3f> &e) const {
        std::vector<geometry_msgs::Pose> ps;
        ps.resize(e.size());
        for (std::size_t i = 0; i < e.size(); i++)
            tf::poseEigenToMsg(e[i].cast<double>(), ps[i]);
        this->poses(name, ps);
    }

    void polygons(std::string name, const surface_types::Surface &surface) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.header.stamp = ros::Time();
            m.ns = name;
            m.id = surface.id;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            // m.pose not needed
            m.scale.x = 0.005;
            // m.scale otherwise not needed
            m.color = surface.color;
            for (auto polygon : surface.polygons) {
                auto first = m.points.size();
                // First add two copies of each point
                for (auto index : polygon.vertices) {
                    geometry_msgs::Point p;
                    p.x = surface.inliers[index].x;
                    p.y = surface.inliers[index].y;
                    p.z = surface.inliers[index].z;
                    m.points.push_back(p);
                    m.points.push_back(p);
                }
                // Then rotate all the new points
                std::rotate(m.points.begin() + first, m.points.begin() + first + 1, m.points.end());
            }

            pub.publish(m);

        });
    }

    void mesh(std::string name, const surface_types::Surface &surface) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame_;
            marker.ns = name;
            marker.id = surface.id;
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            marker.color = surface.color;

            marker.points.reserve(surface.mesh.triangles.size() * 3);
            for (const shape_msgs::MeshTriangle &triangle : surface.mesh.triangles) {
                marker.points.push_back(surface.mesh.vertices[triangle.vertex_indices[0]]);
                marker.points.push_back(surface.mesh.vertices[triangle.vertex_indices[1]]);
                marker.points.push_back(surface.mesh.vertices[triangle.vertex_indices[2]]);
            }

            pub.publish(marker);
        });
    }

    void plane_normal(std::string name, surface_types::Surface &surface) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame_;
            marker.ns = name;
            marker.id = surface.id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 0.05; // Shaft diameter
            marker.scale.y = 0.1;  // Head diameter
            marker.scale.z = 0;    // Head length, or 0 for default
            marker.color = surface.color;

            marker.points.resize(2);
            // Start point: d distance along the normal direction
            marker.points.front().x = -surface.model.values[3] * surface.model.values[0];
            marker.points.front().y = -surface.model.values[3] * surface.model.values[1];
            marker.points.front().z = -surface.model.values[3] * surface.model.values[2];
            // End point: d + 0.1 distance along the normal direction
            marker.points.back().x = (-surface.model.values[3] + 0.5) * surface.model.values[0];
            marker.points.back().y = (-surface.model.values[3] + 0.5) * surface.model.values[1];
            marker.points.back().z = (-surface.model.values[3] + 0.5) * surface.model.values[2];

            pub.publish(marker);
        });
    }

    void pose(std::string name, const surface_types::Surface &surface) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame_;
            marker.ns = name;
            marker.id = surface.id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 0.05; // Shaft diameter
            marker.scale.y = 0.1;  // Head diameter
            marker.scale.z = 0;    // Head length, or 0 for default
            marker.color = surface.color;

            marker.points.resize(2);
            // Start point: d distance along the normal direction
            marker.points.front().x = surface.pose.translation()[0];
            marker.points.front().y = surface.pose.translation()[1];
            marker.points.front().z = surface.pose.translation()[2];
            // End point: d + 0.1 distance along the normal direction
            auto unit = surface.pose * Eigen::Vector3d(0, 0, 0.5);
            marker.points.back().x = unit[0];
            marker.points.back().y = unit[1];
            marker.points.back().z = unit[2];

            pub.publish(marker);

            // Publish the x axis in red
            marker.ns = name + "_x";
            marker.scale.x /= 2;
            marker.scale.y /= 2;
            marker.scale.z /= 2;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 1;
            // End point: d + 0.1 distance along the normal direction
            auto unitx = surface.pose * Eigen::Vector3d(0.25, 0, 0);
            marker.points.back().x = unitx[0];
            marker.points.back().y = unitx[1];
            marker.points.back().z = unitx[2];

            pub.publish(marker);

            // Publish the y axis in red
            marker.ns = name + "_y";
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.color.a = 1;
            // End point: d + 0.1 distance along the normal direction
            auto unity = surface.pose * Eigen::Vector3d(0, 0.25, 0);
            marker.points.back().x = unity[0];
            marker.points.back().y = unity[1];
            marker.points.back().z = unity[2];

            pub.publish(marker);
        });
    }

    void bounding_box(const Eigen::Affine3f &center, const Eigen::Vector3f &extents) {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_;
            m.header.stamp = ros::Time();
            m.ns = "bounding_box";
            m.id = 0;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            tf::poseEigenToMsg(center.cast<double>(), m.pose);
            m.scale.x = 0.01;
            // m.scale otherwise not needed
            m.color.r = 1;
            m.color.g = 1;
            m.color.b = 1;
            m.color.a = 1;

            geometry_msgs::Point pt;
            pt.x = extents[0];
            pt.y = extents[1];
            pt.z = extents[2];

            // Top  4 lines
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.x *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.x *= -1;
            m.points.push_back(pt);

            // Bottom 4 lines
            pt.z *= -1;
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.x *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            m.points.push_back(pt);
            pt.x *= -1;
            m.points.push_back(pt);

            // Side 4 lines
            m.points.push_back(pt);
            pt.z *= -1;
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            pt.z *= -1;
            m.points.push_back(pt);
            pt.x *= -1;
            m.points.push_back(pt);
            pt.z *= -1;
            m.points.push_back(pt);
            pt.y *= -1;
            m.points.push_back(pt);
            pt.z *= -1;
            m.points.push_back(pt);

            pub.publish(m);
        });
    }

    void marker(const visualization_msgs::Marker &marker) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker m = marker;
            m.header.stamp = ros::Time::now();
            m.header.frame_id = frame_;

            pub.publish(m);
        });
    }

    void remove_marker(std::string ns, int32_t id) const {
        with_marker_publisher([=](const ros::Publisher &pub) {
            visualization_msgs::Marker dm;
            dm.action = visualization_msgs::Marker::DELETE;
            dm.ns = ns;
            dm.id = id;

            pub.publish(dm);
        });
    }

protected:
    NHPtr nh_ptr_;
    std::string frame_;
    mutable std::shared_ptr<PubMap> pubs_;
};

#endif // PROJECT_PROGRESSLISTENER_HPP
