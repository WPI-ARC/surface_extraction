#! /usr/bin/env python

import rospy
from collections import OrderedDict

from sensor_msgs.msg import PointCloud2

clouds = OrderedDict()

def first_callback(data):
    pass

def main():
    rospy.Subscriber('/surface_extraction/sac_segmentation/output', PointCloud2, first_callback)

if __name__ == '__main__':
    main()