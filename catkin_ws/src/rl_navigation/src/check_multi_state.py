#!/usr/bin/env python

import rospy
from duckiepond_vehicle.msg import UsvDrive
from sensor_msgs.msg import NavSatFix, Imu
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PID import PID_control
from dynamic_reconfigure.server import Server
from control.cfg import ang_PIDConfig, dis_PIDConfig
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import math
import tf
import numpy as np
import os
import pickle as pkl


class BoatState(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" % self.node_name)

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)

        # setup publisher
        self.sub_p3d1 = rospy.Subscriber(
            "/boat1/p3d_odom", Odometry, self.cb_boat1_odom, queue_size=1)

        self.sub_p3d1 = rospy.Subscriber(
            "/boat2/p3d_odom", Odometry, self.cb_boat2_odom, queue_size=1)

        self.sub_p3d1 = rospy.Subscriber(
            "/boat3/p3d_odom", Odometry, self.cb_boat3_odom, queue_size=1)

        self.sub_p3d1 = rospy.Subscriber(
            "/boat4/p3d_odom", Odometry, self.cb_boat4_odom, queue_size=1)

        # initiallize boat status
        self.boat_odom = [Odometry() for i in range(4)]
        self.yaw = [0 for i in range(4)]

        self.position = []
        self.goal = []
        self.random_init()

        self.velocity_detect = [[0, 0] for i in range(4)]

        # timer
        self.timer = rospy.Timer(rospy.Duration(0.2), self.cb_hrvo)

    def random_init(self):
        self.goal = []
        for i in range(4):
            state, g = self.get_initial_state("boat%d" % (i+1), i)
            self.reset_model(state)
            self.goal.append(g)

    def get_initial_state(self, name, id):
        # start position
        state_msg = ModelState()
        state_msg.model_name = name

        r = 8
        degree = np.random.uniform(-30, 30) + id*90
        yaw = math.radians(degree)

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw-np.pi)
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]

        state_msg.pose.position.x = r * math.cos(yaw)
        state_msg.pose.position.y = r * math.sin(yaw)
        state_msg.pose.position.z = 0

        goal = [r * -math.cos(yaw), r * -math.sin(yaw)]

        return state_msg, goal

    def cb_hrvo(self, event):

        epoch = 0
        iteration = 60
        record = np.zeros([iteration])

        for i in range(iteration):
            rospy.sleep(0.5)
            while True:
                self.update_all()

                min_dis, done = self.check_state()
                # print min_dis
                if min_dis < 1.7:
                    self.random_init()
                    record[epoch] = 0
                    break
                if done:
                    self.random_init()
                    record[epoch] = 1
                    break

            print epoch, record[epoch]

            epoch += 1

        folder = os.getcwd()
        record_name = 'rdpg_multi.pkl'
        fileObject = open(folder+"/"+record_name, 'wb')

        pkl.dump(record, fileObject)
        fileObject.close()
        exit()

    def check_state(self):
        min_dis = 1e9
        done = True
        for i in range(4):
            p = np.array(self.position[i])
            pos_dis = np.linalg.norm(p)
            done = True if pos_dis > 10 else False
            for k in range(i+1, 4):
                p1 = np.array(self.position[i])
                p2 = np.array(self.position[k])
                dis = np.linalg.norm(p1-p2)
                min_dis = dis if dis < min_dis else min_dis
        return min_dis, done

    def update_all(self):
        self.position = []
        for i in range(4):
            # update position
            pos = [self.boat_odom[i].pose.pose.position.x,
                   self.boat_odom[i].pose.pose.position.y]
            self.position.append(pos)

            # update orientation
            quaternion = (self.boat_odom[i].pose.pose.orientation.x,
                          self.boat_odom[i].pose.pose.orientation.y,
                          self.boat_odom[i].pose.pose.orientation.z,
                          self.boat_odom[i].pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw[i] = euler[2]

            # update velocity
            self.velocity_detect[i] = [self.boat_odom[i].twist.twist.linear.x,
                                       self.boat_odom[i].twist.twist.linear.y]

    def cb_boat1_odom(self, msg):
        self.boat_odom[0] = msg

    def cb_boat2_odom(self, msg):
        self.boat_odom[1] = msg

    def cb_boat3_odom(self, msg):
        self.boat_odom[2] = msg

    def cb_boat4_odom(self, msg):
        self.boat_odom[3] = msg


if __name__ == "__main__":
    rospy.init_node("BoatState")
    boat_state = BoatState()
    rospy.spin()
