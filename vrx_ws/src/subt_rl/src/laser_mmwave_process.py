#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np
import time


class Process(object):
    def __init__(self):
        self.use_mmwave = rospy.get_param('~use_mmwave', False)

        self.laser_mmwave = None

        self.sub_laser_upper = rospy.Subscriber(
            'RL/scan', LaserScan, self.cb_laser, queue_size=1)
        self.sub_mm_laser = rospy.Subscriber(
            'RL/scan_mmwave', LaserScan, self.cb_laser_mmwave, queue_size=1)

        self.pub_sc = rospy.Publisher(
            'RL/process', LaserScan, queue_size=1)

        rospy.loginfo("laser_preprocess init")

    def cb_laser(self, msg):
        if self.laser_mmwave is None:
            self.pub_sc.publish(msg)
            rospy.loginfo("mmwave laser not received")
            return


        l1 = list(msg.ranges)
        l2 = list(self.laser_mmwave.ranges)
        sc_msg = msg
        sc_msg.ranges = []

        for i, (d1,d2) in enumerate(zip(l1,l2)):
            sc_msg.ranges.append(min(d1,d2))
        
        # print len(sc_msg.ranges)
        self.pub_sc.publish(sc_msg)
        self.laser_mmwave = None

    def cb_laser_mmwave(self, msg):
        self.laser_mmwave = msg

    def on_shutdown(self):
        pass


if __name__ == "__main__":
    rospy.init_node('laser_mmwave_process')
    process = Process()
    rospy.on_shutdown(process.on_shutdown)
    rospy.spin()
