#! /usr/bin/env python
import rospy
import tensorflow as tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import os
from PID import PID_control
import time


class RunDDPGModel(object):
    def __init__(self):
        self.sim = rospy.get_param('~sim', True)
        self.version = rospy.get_param('~version', 0)
        self.model = rospy.get_param('~model', 'frozen_model.pb')
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
        self.sess = tf.Session(graph=self.graph,
                               config=config)

        self.action_bound = {}
        self.topic_name = ''
        if not self.sim:
            self.topic_name = '/husky_velocity_controller/cmd_vel'
            self.action_bound = {'linear': 0.3, 'angular': 0.35}
        else:
            self.topic_name = '/X1/cmd_vel'
            self.action_bound = {'linear': 1.5, 'angular': 0.8}

        self.pub_twist = rospy.Publisher(self.topic_name, Twist, queue_size=1)
        self.laser_upper = LaserScan()
        self.laser_lower = LaserScan()

        self.sub_laser_upper = rospy.Subscriber(
            '/RL/scan/upper', LaserScan, self.cb_laser_upper, queue_size=1)
        self.sub_laser_lower = rospy.Subscriber(
            '/RL/scan/lower', LaserScan, self.cb_laser_lower, queue_size=1)

        self.input = self.graph.get_tensor_by_name('state:0')
        self.output_q = self.graph.get_tensor_by_name(
            'sample_action/Squeeze:0')
        
        rospy.loginfo('preparing')
        time.sleep(1)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_pub)

    def scale_angular(self, n, bound):
        return max(min(n, 1), -1)*bound

    def scale_linear(self, n, bound):
        return ((n+1)/2)*bound

    def timer_pub(self, event):
        laser = self.get_observation()
        if len(laser) != 0:
            laser = laser[np.newaxis, :]  # batch size
            action_out = self.sess.run(self.output_q, feed_dict={self.input: laser})
            action_out = np.squeeze(action_out)
            action_out = np.clip(action_out, -1, 1)
            
            cmd_vel = Twist()
            cmd_vel.linear.x = self.scale_linear(action_out[0], self.action_bound['linear'])
            cmd_vel.angular.z = self.scale_angular(action_out[1], self.action_bound['angular'])

            print(cmd_vel.linear.x, cmd_vel.angular.z)
            self.pub_twist.publish(cmd_vel)

    def get_observation(self):
        laser = []
        max_dis = 1.5
        for i, dis in enumerate(list(self.laser_upper.ranges)):
            if dis > max_dis:
                dis = max_dis
            laser.append(dis)
        for i, dis in enumerate(list(self.laser_lower.ranges)):
            if dis > max_dis:
                dis = max_dis
            laser.append(dis)
        return np.array(laser)

    def cb_laser_upper(self, msg):
        self.laser_upper = msg

    def cb_laser_lower(self, msg):
        self.laser_lower = msg

    def on_shutdown(self):
        self.sess.close()


if __name__ == '__main__':
    rospy.init_node('rl_ppo')
    runmodel = RunDDPGModel()
    rospy.on_shutdown(runmodel.on_shutdown)
    rospy.spin()
