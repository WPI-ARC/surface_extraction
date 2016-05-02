#! /usr/bin/env python

from __future__ import print_function, division

import rospy
import dynamic_reconfigure.client

if True or __name__ == "__main__":
    print("Starting node...")
    rospy.init_node('emit_current_time')
    print("Started")

    client = dynamic_reconfigure.client.Client('filter')

    secs_to_skip = 3
    speedup_factor = 1 # 0.20
    window_size = 5 # seconds

    start = rospy.get_time() - secs_to_skip

    rate = rospy.Rate(10)  # Hz


    print("Starting filter_limit expansion")

    while not rospy.is_shutdown():
        limit_max = (rospy.get_time() - start) * speedup_factor
        limit_min = (rospy.get_time() - start - window_size) * speedup_factor
        try:
            config = client.update_configuration({'filter_limit_max': limit_max,
                                                  'filter_limit_min': limit_min})
        except rospy.service.ServiceException:
            pass
        else:
            print("Limiting filter value to between {} and {}".format(limit_min, limit_max))

        rate.sleep()