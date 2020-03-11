#! /usr/bin/env python
import rospy
import tensorflow as tf
from sensor_msgs.msg import LaserScan, Joy, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import os
from PID import PID_control
import time


class RunDDPGModel(object):
    def __init__(self):
        self.sim = rospy.get_param('~sim', False)
        self.model = rospy.get_param('~model', 'ddpg1214.pb')
        self.input_scan = rospy.get_param('~input_name', 's_:0')
        self.output_name = rospy.get_param(
            '~output_name', 'Actor/target/action_out:0')
        self.auto = False

        self.graph = tf.Graph()
        my_dir = os.path.abspath(os.path.dirname(__file__))
        PATH_TO_CKPT = os.path.join(
            my_dir, "../model/" + self.model)
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(graph=self.graph, config=config)

        self.scan_tensor = self.graph.get_tensor_by_name(self.input_scan)
        self.output_tensor = self.graph.get_tensor_by_name(self.output_name)
        self.action_bound = {'linear': 1.5, 'angular': 0.8} if self.sim else {
            'linear': 0.3, 'angular': 0.3}

        self.scan = None
        self.goal = 0
        self.heading = 0
        self.sub_sc_process = rospy.Subscriber(
            '/RL/scan', LaserScan, self.cb_sc, queue_size=1)

        self.sub_joy = rospy.Subscriber('joy_teleop/joy', Joy, self.cb_joy, queue_size=1)
        pub_name = '/husky_velocity_controller/cmd_vel'
        self.pub_twist = rospy.Publisher(pub_name, Twist, queue_size=1)

        time.sleep(1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_pub)
        rospy.loginfo('press start')

    def cb_sc(self, msg):
        self.scan = np.array(msg.ranges)

    def scale(self, n, bound):
        return max(min(n, 1), -1)*bound

    def timer_pub(self, event):
        if self.scan is not None and self.auto:
            laser = self.scan[np.newaxis, :]  # batch size
            action_out = self.sess.run(
                self.output_tensor, feed_dict={self.scan_tensor: laser})
            action_out = np.squeeze(action_out)
            
            cmd_vel = Twist()
            cmd_vel.linear.x = self.scale(
                action_out[0], self.action_bound['linear'])
            cmd_vel.angular.z = self.scale(
                action_out[1], self.action_bound['angular'])

            print cmd_vel.linear.x,cmd_vel.angular.z
            # print(cmd_vel.linear.x, cmd_vel.angular.z)
            self.pub_twist.publish(cmd_vel)

    def on_shutdown(self):
        self.sess.close()

    def cb_joy(self, joy_msg):
        # MODE D
        start_button = 9 if self.sim else 7
        back_button = 8 if self.sim else 6
        # Start button
        if (joy_msg.buttons[start_button] == 1):
            self.auto = True
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1:
            self.auto = False
            rospy.loginfo('go manual')


if __name__ == '__main__':
    rospy.init_node('rl_ddpg')
    runmodel = RunDDPGModel()
    rospy.on_shutdown(runmodel.on_shutdown)
    rospy.spin()
