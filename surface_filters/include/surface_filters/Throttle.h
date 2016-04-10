//
// Created by will on 4/9/16.
//

#ifndef SURFACE_FILTERS_THROTTLEDTIMER_H
#define SURFACE_FILTERS_THROTTLEDTIMER_H

// System and Boost includes
#include <mutex>

// ROS includes
#include <ros/timer.h>

namespace surface_filters {
class Throttle {
private:
    ros::Duration interval_;
    ros::Time last_run_time_;
    bool has_queued_run_;
    std::mutex mutex_;

public:
    Throttle(ros::Duration interval) : interval_(interval), last_run_time_(0), has_queued_run_(false), mutex_() {}

    Throttle() : interval_(0), last_run_time_(0), has_queued_run_(false), mutex_() {}

    bool runNow() {
        assert(interval_ > ros::Duration(0));

        std::unique_lock<std::mutex> lock(mutex_);

        // Called while another run is already queued: don't run
        if (has_queued_run_) return false;

        auto now = ros::Time::now();
        auto next_run_time = last_run_time_ + interval_;

        // First time called, or first in a while: immediately run and update the last run time
        if (next_run_time <= now) {
            last_run_time_ = now;
            return true;
        }

        // Called while nothing is queued but there has already been a recent run: queue this call
        has_queued_run_ = true;

        // If the lock was held while waiting, it would block every call to runNow and probably exhaust all threads
        lock.unlock();

        ros::Time::sleepUntil(next_run_time);

        // Re-assert the lock
        // Despite not holding the lock during the sleep, there's a guarantee that no thread ran because of the
        // check on has_queued_run_.
        lock.lock();

        has_queued_run_ = false;
        last_run_time_ = ros::Time::now(); // Don't use next_run_time because sleepUntil may have slept longer
        return true;
    }

    void setInterval(ros::Duration interval) {
        // Lock so interval can't change in the middle of runNow
        std::lock_guard<std::mutex> lock(mutex_);
        interval_ = interval;
    }
};
}

#endif // SURFACE_FILTERS_THROTTLEDTIMER_H
