#!/usr/bin/env python
import math
import rospy
import tf
from tf import transformations as trans

def handle_turtle_pose(msg, turtlename):
    br.sendTransform((msg.x, msg.y, 0),
                     trans.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('animate_tf_broadcaster')
    br = tf.TransformBroadcaster()

    # radius = 0
    # start_quat = trans.quaternion_from_euler(0, 0, 0)
    # end_quat = trans.quaternion_from_euler(math.pi, 0, 0)

    animation_duration_s = 10 # seconds

    animation_start = rospy.Time.now()

    rate = rospy.Rate(100) # Hz

    d_end = 5 # meters

    try:
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - animation_start).to_sec() / animation_duration_s

            if t > 1: break

            br.sendTransform((d_end * t, 0, 0), trans.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "world", "camera")

            print(t)
            rate.sleep()

    except rospy.ROSInterruptException:
          pass