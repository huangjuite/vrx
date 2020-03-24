#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from PID import PID_control
from dynamic_reconfigure.server import Server
from control.cfg import ang_PIDConfig, dis_PIDConfig
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
import pandas as pd
import pickle as pkl
import numpy as np
import os
import gym_env
import random
import time

objs_dict = {
    "red_totem": 0.5,
    "green_totem": 0.5,
    "yellow_totem": 0.5,
    "black_totem": 0.5,
    "blue_totem": 0.5,
    "ball3": 0.3,
    "ball5": 0.5,
    "ball7": 0.7,
}


class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        # initiallize PID
        self.dis_pid = PID_control("distance_control")
        self.angu_pid = PID_control("angular_control")
        self.dis_server = Server(
            dis_PIDConfig, self.cb_dis_pid, "distance_control")
        self.ang_server = Server(
            ang_PIDConfig, self.cb_ang_pid, "angular_control")
        self.dis_pid.setSampleTime(0.1)
        self.angu_pid.setSampleTime(0.1)
        self.dis_pid.SetPoint = 0
        self.angu_pid.SetPoint = 0

        # setup publisher
        # self.pub_v1 = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # self.sub_p3d1 = rospy.Subscriber(
        #     "p3d_odom", Odometry, self.cb_boat1_odom, queue_size=1)

        # initiallize boat status
        self.boat_odom = Odometry()
        self.yaw = 0

        # initiallize HRVO environment
        self.ws_model = dict()
        # robot radius
        self.ws_model['robot_radius'] = 1

        print os.path.dirname(__file__)
        data = pd.read_csv(
            "/home/developer/vrx/catkin_ws/src/vrx/vrx_gazebo/worlds/block_position.csv")
        objs = []
        for idx, item in data.iterrows():
            objs.append([item['x'], item['y'], objs_dict[item['object']]])
        self.ws_model['circular_obstacles'] = objs

        # rectangular boundary, format [x,y,width/2,heigth/2]
        self.ws_model['boundary'] = []

        self.position = [[0, 0]]
        self.goal = self.random_goal()
        # print(self.position)
        # print(self.goal)
        self.velocity = [[0, 0]]
        self.v_max = [1]

        # timer
        # self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_hrvo)

    def random_goal(self):
        # random angle
        alpha = 2 * math.pi * random.random()
        # random radius
        r = 50 * math.sqrt(2)
        # calculating coordinates
        x = r * math.cos(alpha)
        y = r * math.sin(alpha)

        return [[x, y]]

    def start_hrvo(self):
        env = gym_env.SubtCaveNobackEnv()

        epoch = 0
        iteration = 60
        record = np.zeros([iteration, 2])

        for i in range(iteration):
            start_time = time.time()
            ep_reward = 0
            step = 0
            distance = 0
            while True:

                v_des = compute_V_des(self.position, self.goal, self.v_max)
                self.velocity = RVO_update(
                    self.position, v_des, self.velocity, self.ws_model)

                dis, angle = self.process_ang_dis(
                    self.velocity[0][0], self.velocity[0][1], self.yaw)

                self.position, self.yaw, out_bound, r, done, info = env.step(
                    [dis * 0.4, angle * 0.4])

                if out_bound or done:
                    self.goal = self.random_goal()

                if done or env.total_dis > 600:
                    record[epoch][0] = env.total_dis
                    record[epoch][1] = time.time()-start_time
                    s = env.reset()
                    break
            print epoch, record[epoch]

            epoch += 1

        folder = os.getcwd()
        record_name = 'hrvo.pkl'
        fileObject = open(folder+"/"+record_name, 'wb')

        pkl.dump(record, fileObject)
        fileObject.close()

    def process_ang_dis(self, vx, vy, yaw):
        dest_yaw = math.atan2(vy, vx)

        angle = dest_yaw - yaw
        if angle > np.pi:
            angle = angle-2*np.pi

        if angle < -np.pi:
            angle = angle+2*np.pi

        angle = angle/np.pi

        dis = math.sqrt(vx**2+vy**2)

        # print "pos      %2.2f, %2.2f" % (self.position[0][0], self.position[0][1])
        # print "goal     %2.2f, %2.2f" % (self.goal[0][0], self.goal[0][1])
        # print "dest_yaw %2.2f" % dest_yaw
        # print "yaw      %2.2f" % yaw
        # print "angle    %2.2f" % angle
        # print "dis      %2.2f\n" % dis

        dis = max(min(dis, 1), -1)
        angle = max(min(angle, 1), -1)
        return dis, angle

    def update_all(self):
        self.position = []

        # update position
        pos = [self.boat_odom.pose.pose.position.x,
               self.boat_odom.pose.pose.position.y]
        self.position.append(pos)

        # update orientation
        quaternion = (self.boat_odom.pose.pose.orientation.x,
                      self.boat_odom.pose.pose.orientation.y,
                      self.boat_odom.pose.pose.orientation.z,
                      self.boat_odom.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def cb_boat1_odom(self, msg):
        self.boat_odom = msg

    def cb_dis_pid(self, config, level):
        print(
            "distance: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.dis_pid.setKp(Kp)
        self.dis_pid.setKi(Ki)
        self.dis_pid.setKd(Kd)
        return config

    def cb_ang_pid(self, config, level):
        print(
            "angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        self.angu_pid.setKp(Kp)
        self.angu_pid.setKi(Ki)
        self.angu_pid.setKd(Kd)
        return config


if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    boatHRVO.start_hrvo()
    rospy.spin()
