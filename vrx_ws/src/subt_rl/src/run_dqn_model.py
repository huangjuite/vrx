#! /usr/bin/env python
import rospy
import tensorflow as tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import os
from PID import PID_control


class RunModel(object):
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

        self.actions = []
        self.linear_pid = PID_control(p_name='linear', P=1, I=0, D=0)
        self.angular_pid = PID_control(p_name='angular', P=1, I=0, D=0)
        self.topic_name = ''
        if not self.sim:
            self.linear_pid = PID_control(p_name='linear', P=1, I=0, D=0)
            self.angular_pid = PID_control(p_name='angular', P=1, I=0, D=0)
            self.topic_name = '/husky_velocity_controller/cmd_vel'
            if self.version == 0:
                self.actions = [[0.1, -0.35],
                                [0.3, -0.35],
                                [0.3, -0.2],
                                [0.3, 0.0],
                                [0.3, 0.2],
                                [0.3, 0.35],
                                [0.1, 0.35]]
            elif self.version == 1:
                max_ang_speed = 0.4
                max_linear = 0.35
                for action in range(21):
                    ang_vel = (action-10)*max_ang_speed*0.1
                    lin_vel = max_linear-((action-10)/2.6)**2*0.1
                    self.actions.append([lin_vel, ang_vel])

        else:
            self.linear_pid = PID_control(p_name='linear', P=0.8, I=0, D=0)
            self.angular_pid = PID_control(p_name='angular', P=0.8, I=0, D=0)
            self.topic_name = '/X1/cmd_vel'
            if self.version == 0:
                self.actions = [[0.5, -0.8],
                                [1.5, -0.8],
                                [1.5, -0.4],
                                [1.5, 0.0],
                                [1.5, 0.4],
                                [1.5, 0.8],
                                [0.5, 0.8]]
            elif self.version == 1:
                max_ang_speed = 0.8
                for action in range(21):
                    ang_vel = (action-10)*max_ang_speed*0.1
                    lin_vel = 1.5-((action-10)/2.6)**2*0.1
                    self.actions.append([lin_vel, ang_vel])

        self.pub_twist = rospy.Publisher(self.topic_name, Twist, queue_size=1)
        self.laser_upper = LaserScan()
        self.laser_lower = LaserScan()

        self.sub_laser_upper = rospy.Subscriber(
            '/RL/scan/upper', LaserScan, self.cb_laser_upper, queue_size=1)
        self.sub_laser_lower = rospy.Subscriber(
            '/RL/scan/lower', LaserScan, self.cb_laser_lower, queue_size=1)

        self.input = self.graph.get_tensor_by_name('eval_net/input/s:0')
        self.output_q = self.graph.get_tensor_by_name(
            'eval_net/l3/output_q:0')

        self.last_cmd = [0, 0]
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_pub)

    def timer_pub(self, event):
        laser = self.get_observation()
        if len(laser) != 0:
            laser = laser[np.newaxis, :]  # batch size
            q_out = self.sess.run(self.output_q, feed_dict={self.input: laser})
            action = np.argmax(q_out)
            cmd_vel = Twist()
            self.linear_pid.update(self.last_cmd[0] - self.actions[action][0])
            self.angular_pid.update(self.last_cmd[1] - self.actions[action][1])
                
            cmd_vel.linear.x = self.linear_pid.output + self.last_cmd[0]
            cmd_vel.angular.z = self.angular_pid.output + self.last_cmd[1]
            self.last_cmd[0] = cmd_vel.linear.x
            self.last_cmd[1] = cmd_vel.angular.z

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
    rospy.init_node('rl_dqn')
    runmodel = RunModel()
    rospy.on_shutdown(runmodel.on_shutdown)
    rospy.spin()
