//
// Created by will on 6/21/16.
//

#ifndef PROJECT_PROGRESSLISTENER_HPP
#define PROJECT_PROGRESSLISTENER_HPP

#include <ros/ros.h>

class ProgressListener {
    typedef std::map<std::string, ros::Publisher> PubMap;

public:
    // Use this constructor to disable ProgressListener
    ProgressListener() : nh_ptr_(nullptr), pubs_(nullptr), frame_("") {}
    ProgressListener(ros::NodeHandle *np, std::string frame)
        : nh_ptr_(std::make_shared<ros::NodeHandle *>(np)), pubs_(std::make_shared<PubMap>()), frame_(frame) {}

    template <typename PointT>
    void points(std::string name, const typename pcl::PointCloud<PointT>::ConstPtr &points) {
        if (!nh_ptr_) return;

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
        auto points = boost::make_shared<pcl::PointCloud<PointT>>(pair.first, pair.second.indices);
        points->header.frame_id = frame_;
        this->points<PointT>(name, points);
    }

protected:
    std::shared_ptr<ros::NodeHandle *> nh_ptr_;
    std::shared_ptr<PubMap> pubs_;
    std::string frame_;
};

#endif // PROJECT_PROGRESSLISTENER_HPP
