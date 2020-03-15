#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np
import time


class Process(object):
    def __init__(self):
        self.use_mmwave = rospy.get_param('~use_mmwave', False)

        self.laser_upper = None
        self.laser_mid = None
        self.laser_lower = None
        self.laser_mmwave = None

        shape = [3, 315]
        self.layout = MultiArrayLayout()
        self.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
        self.layout.dim[0].label = "height"
        self.layout.dim[0].size = shape[0]
        self.layout.dim[0].stride = shape[0]*shape[1]

        self.layout.dim[1].label = "width"
        self.layout.dim[1].size = shape[1]
        self.layout.dim[1].stride = shape[1]


        self.sub_laser_upper = rospy.Subscriber(
            '/RL/scan/upper', LaserScan, self.cb_laser_upper, queue_size=1)
        self.sub_laser_mid = rospy.Subscriber(
            '/RL/scan/mid', LaserScan, self.cb_laser_mid, queue_size=1)
        self.sub_laser_lower = rospy.Subscriber(
            '/RL/scan/lower', LaserScan, self.cb_laser_lower, queue_size=1)
        self.sub_mm_laser = rospy.Subscriber(
            '/RL/scan/mmwave', LaserScan, self.cb_laser_mmwave, queue_size=1)

        self.pub_sc = rospy.Publisher(
            '/RL/process', Float32MultiArray, queue_size=1)

        time.sleep(0.5)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_pub)

        rospy.loginfo("laser_preprocess init")

    def timer_pub(self, event):
        if self.laser_lower is None or self.laser_mid is None or self.laser_lower is None:
            return

        level1 = list(self.laser_upper.ranges)
        level2 = list(self.laser_mid.ranges)
        level3 = list(self.laser_lower.ranges)

        sc_array = None
        max_dis = 5

        if self.use_mmwave and self.laser_mmwave is not None:
            mmwave = list(self.laser_mmwave.ranges)
            for i, (l1, l2, l3, mm) in enumerate(zip(level1, level2, level3, mmwave)):
                sc = [[l1], [l2], [l3]]
                sc = np.clip(sc, 0, min(max_dis, mm))
                if sc_array is None:
                    sc_array = sc
                else:
                    sc_array = np.hstack((sc_array, sc))
        else:
            for i, (l1, l2, l3) in enumerate(zip(level1, level2, level3)):
                sc = [[l1], [l2], [l3]]
                sc = np.clip(sc, 0, max_dis)
                if sc_array is None:
                    sc_array = sc
                else:
                    sc_array = np.hstack((sc_array, sc))

        # print sc_array.shape
        sc_msg = Float32MultiArray(layout=self.layout,data=sc_array.reshape([3*315]))
        self.pub_sc.publish(sc_msg)

    def cb_laser_upper(self, msg):
        self.laser_upper = msg

    def cb_laser_mid(self, msg):
        self.laser_mid = msg

    def cb_laser_lower(self, msg):
        self.laser_lower = msg

    def cb_laser_mmwave(self, msg):
        self.laser_mmwave = msg

    def on_shutdown(self):
        pass


if __name__ == "__main__":
    rospy.init_node('pc_to_laser')
    process = Process()
    rospy.on_shutdown(process.on_shutdown)
    rospy.spin()
