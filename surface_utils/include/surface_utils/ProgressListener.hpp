//
// Created by will on 6/21/16.
//

#ifndef PROJECT_PROGRESSLISTENER_HPP
#define PROJECT_PROGRESSLISTENER_HPP

#include <ros/ros.h>
// pcl_ros/point_cloud.h enables publishing topics using PCL types
#include <pcl_ros/point_cloud.h>
#include <thread>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <surface_types/Surface.hpp>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Affine3d)

class ProgressListener {
    typedef std::map<std::string, ros::Publisher> PubMap;
    typedef std::shared_ptr<ros::NodeHandle *> NHPtr;

public:
    // Use this constructor to disable ProgressListener
    ProgressListener() : nh_ptr_(nullptr), pubs_(nullptr), frame_("") {}
    ProgressListener(ros::NodeHandle *np, std::string frame)
        : nh_ptr_(std::make_shared<ros::NodeHandle *>(np)), pubs_(std::make_shared<PubMap>()), frame_(frame) {}

    template <typename PointT>
    void points(std::string name, const typename pcl::PointCloud<PointT>::Ptr &points) {
        if (!nh_ptr_) return;
        points->header.frame_id = frame_;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(
            std::bind([name](NHPtr nh, std::shared_ptr<PubMap> pubs, typename pcl::PointCloud<PointT>::ConstPtr pts) {
                assert(name.length() != 0);
                assert(nh != nullptr);
                assert(pubs != nullptr);
                assert(pts != nullptr);

                std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

                if (pub == pubs->end()) {
                    pub = pubs->insert(std::make_pair(name, (*nh)->advertise<pcl::PointCloud<PointT>>(name, 10))).first;

                    ros::Rate r(2);
                    ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                    while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                        r.sleep();
                    }
                }

                pub->second.publish(pts);

            }, nh_ptr_, pubs_, points)).detach();
    }

    template <typename PointT>
    void pair(std::string name, typename std::pair<pcl::PointCloud<PointT>, pcl::PointIndices> pair) {
        if (!nh_ptr_) return;

        auto points = boost::make_shared<pcl::PointCloud<PointT>>(pair.first, pair.second.indices);
        this->points<PointT>(name, points);
    }

    void normal_vectors(std::string name, pcl::PointCloud<pcl::PointNormal>::Ptr &normals_cloud) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind(
                        [name](NHPtr nh, std::shared_ptr<PubMap> pubs, pcl::PointCloud<pcl::PointNormal>::ConstPtr pts,
                               std::string frame) {
                            assert(name.length() != 0);
                            assert(nh != nullptr);
                            assert(pubs != nullptr);
                            assert(pts != nullptr);

                            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

                            bool made_new_pub = false;
                            if (pub == pubs->end()) {
                                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<geometry_msgs::PoseArray>(
                                                                            name, 10))).first;
                                made_new_pub = true;
                            }

                            geometry_msgs::PoseArray poses;
                            poses.header.frame_id = frame;

                            Eigen::Affine3d ztf(Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitY()));

                            for (auto &pt : *pts) {
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

                            if (made_new_pub) { // Then wait for it
                                ros::Rate r(2);
                                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                                    r.sleep();
                                }
                            }

                            pub->second.publish(poses);
                        },
                        nh_ptr_, pubs_, normals_cloud, frame_)).detach();
    }

    void pose(std::string name, geometry_msgs::PoseStamped &p) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, p](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<geometry_msgs::PoseStamped>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            pub->second.publish(p);

        }, nh_ptr_, pubs_, frame_)).detach();
    }

    void pose(std::string name, geometry_msgs::Pose &p) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = frame_;
        ps.pose = p;
        this->pose(name, ps);
    }

    void pose(std::string name, Eigen::Affine3d &e) {
        geometry_msgs::Pose p;
        tf::poseEigenToMsg(e, p);
        this->pose(name, p);
    }

    void pose(std::string name, Eigen::Affine3f &e) {
        geometry_msgs::Pose p;
        tf::poseEigenToMsg(e.cast<double>(), p);
        this->pose(name, p);
    }

    void poses(std::string name, std::vector<geometry_msgs::Pose> &p) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, p](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<geometry_msgs::PoseArray>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            geometry_msgs::PoseArray ps;
            ps.header.frame_id = frame;
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

            pub->second.publish(ps);

        }, nh_ptr_, pubs_, frame_)).detach();
    }

    void poses(std::string name, std::vector<Eigen::Affine3d> &e) {
        std::vector<geometry_msgs::Pose> ps;
        ps.resize(e.size());
        for (std::size_t i = 0; i < e.size(); i++)
            tf::poseEigenToMsg(e[i], ps[i]);
        this->poses(name, ps);
    }

    void poses(std::string name, std::vector<Eigen::Affine3f> &e) {
        std::vector<geometry_msgs::Pose> ps;
        ps.resize(e.size());
        for (std::size_t i = 0; i < e.size(); i++)
            tf::poseEigenToMsg(e[i].cast<double>(), ps[i]);
        this->poses(name, ps);
    }

    void polygons(std::string name, surface_types::Surface &surface) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, surface](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<visualization_msgs::Marker>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            visualization_msgs::Marker m;
            m.header.frame_id = frame;
            m.header.stamp = ros::Time();
            m.ns = "surface_polygons";
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

            pub->second.publish(m);
        }, nh_ptr_, pubs_, frame_)).detach();
    }

    void mesh(std::string name, surface_types::Surface &surface) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, surface](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<visualization_msgs::Marker>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame;
            marker.ns = "surface_triangles";
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

            pub->second.publish(marker);
        }, nh_ptr_, pubs_, frame_)).detach();
    }

    void plane_normal(std::string name, surface_types::Surface &surface) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, surface](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<visualization_msgs::Marker>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame;
            marker.ns = "surface_normals";
            marker.id = surface.id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 0.05; // Shaft diameter
            marker.scale.y = 0.1; // Head diameter
            marker.scale.z = 0; // Head length, or 0 for default
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

            pub->second.publish(marker);
        }, nh_ptr_, pubs_, frame_)).detach();
    }

    void pose(std::string name, surface_types::Surface &surface) {
        if (!nh_ptr_) return;

        // Need another reference to the member data pointers in case object is destroyed before the thread finishes
        std::thread(std::bind([name, surface](NHPtr nh, std::shared_ptr<PubMap> pubs, std::string frame) {
            assert(name.length() != 0);
            assert(nh != nullptr);
            assert(pubs != nullptr);

            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

            if (pub == pubs->end()) {
                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<visualization_msgs::Marker>(name, 10))).first;

                ros::Rate r(2);
                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                    r.sleep();
                }
            }

            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = frame;
            marker.ns = "surface_poses";
            marker.id = surface.id;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::MODIFY;
            // perimeter.pose not needed
            marker.scale.x = 0.05; // Shaft diameter
            marker.scale.y = 0.1; // Head diameter
            marker.scale.z = 0; // Head length, or 0 for default
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

            pub->second.publish(marker);

            // Publish the x axis in red
            marker.ns = "surface_poses_x";
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

            pub->second.publish(marker);

            // Publish the y axis in red
            marker.ns = "surface_poses_y";
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.color.a = 1;
            // End point: d + 0.1 distance along the normal direction
            auto unity = surface.pose * Eigen::Vector3d(0, 0.25, 0);
            marker.points.back().x = unity[0];
            marker.points.back().y = unity[1];
            marker.points.back().z = unity[2];

            pub->second.publish(marker);


        }, nh_ptr_, pubs_, frame_)).detach();
    }

protected:
    NHPtr nh_ptr_;
    std::shared_ptr<PubMap> pubs_;
    std::string frame_;
};

#endif // PROJECT_PROGRESSLISTENER_HPP
