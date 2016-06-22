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
#include <tf/tf.h>

class ProgressListener {
    typedef std::map<std::string, ros::Publisher> PubMap;

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
        std::thread(std::bind(
                        [name](std::shared_ptr<ros::NodeHandle *> nh, std::shared_ptr<PubMap> pubs,
                               typename pcl::PointCloud<PointT>::ConstPtr pts) {
                            assert(name.length() != 0);
                            assert(nh != nullptr);
                            assert(pubs != nullptr);
                            assert(pts != nullptr);

                            std::map<std::string, ros::Publisher>::iterator pub = pubs->find(name);

                            if (pub == pubs->end()) {
                                pub = pubs->insert(std::make_pair(name, (*nh)->advertise<pcl::PointCloud<PointT>>(
                                                                            name, 10))).first;

                                ros::Rate r(2);
                                ros::Time stop_time = ros::Time::now() + ros::Duration(10);
                                while (pub->second.getNumSubscribers() == 0 && ros::Time::now() < stop_time) {
                                    r.sleep();
                                }
                            }

                            pub->second.publish(pts);

                        },
                        nh_ptr_, pubs_, points)).detach();
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
                [name](std::shared_ptr<ros::NodeHandle *> nh, std::shared_ptr<PubMap> pubs,
                       pcl::PointCloud<pcl::PointNormal>::ConstPtr pts, std::string frame) {
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

                        //adding pose to pose array
                        pose.position.x = pt.x;
                        pose.position.y = pt.y;
                        pose.position.z = pt.z;
                        pose.orientation = msg;
                        poses.poses.push_back(pose);
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

protected:
    std::shared_ptr<ros::NodeHandle *> nh_ptr_;
    std::shared_ptr<PubMap> pubs_;
    std::string frame_;
};

#endif // PROJECT_PROGRESSLISTENER_HPP
