/*
 * Created by Will Pryor at the Autonomous Research Collaboration Lab at Worcester Polytechnic Institute
 */


#include <cstdio>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Services
#include <laser_assembler/AssembleScans2.h>

// Messages
#include <sensor_msgs/PointCloud.h>


namespace surface_filters {

    class PeriodicSnapshotter : public nodelet::Nodelet {

    public:

        PeriodicSnapshotter() : first_time_(true) { }

        virtual void onInit() {
            n_ = getPrivateNodeHandle();
            // Create a publisher for the clouds that we assemble
            pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 1);

            // Create the service client for calling the assembler
            client_ = n_.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");

            NODELET_DEBUG("[%s::onInit] Waiting for the service to become available", getName().c_str());
            client_.waitForExistence();
            NODELET_DEBUG("[%s::onInit] Service is available!", getName().c_str());

            // Start the timer that will trigger the processing loop (timerCallback)
            n_.createTimer(ros::Duration(10, 0), &PeriodicSnapshotter::timerCallback, this);

            // Need to track if we've called the timerCallback at least once
            first_time_ = true;
        }

        void timerCallback(const ros::TimerEvent &e) {

            // We don't want to build a cloud the first callback, since we we
            //   don't have a start and end time yet
            if (first_time_) {
                first_time_ = false;
                return;
            }

            // Populate our service request based on our timer callback times
            laser_assembler::AssembleScans2 srv;
            srv.request.begin = e.last_real;
            srv.request.end = e.current_real;

            // Make the service call
            if (client_.call(srv)) {
                pub_.publish(srv.response.cloud);
            }
            else {
                NODELET_INFO_STREAM(
                        "Error making service call with begin=" << srv.request.begin << " and end=" << srv.request.end);
            }
        }

    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::ServiceClient client_;
        bool first_time_;
    };

}

PLUGINLIB_EXPORT_CLASS(surface_filters::PeriodicSnapshotter, nodelet::Nodelet)
