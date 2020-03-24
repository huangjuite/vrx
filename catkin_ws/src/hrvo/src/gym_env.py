
import rospy
import time
import cv2
import numpy as np
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan, Image, CompressedImage, Imu
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# [x,y,z]
INITIAL_STATES = [
    [0, 0, 0]]


class SubtCaveNobackEnv():
    metadata = {'render.modes': ['laser']}

    def __init__(self):
        # rospy.init_node('gym_subt')
        self.laser_upper = LaserScan()
        self.laser_lower = LaserScan()
        self.image = CompressedImage()

        self.action_bound = {'linear': 0.4, 'angular': 0.4}

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_model = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.get_model = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)

        self.pub_twist = rospy.Publisher(
            '/duckie_alpha/cmd_vel', Twist, queue_size=1)

        self.total_dis = 0
        self.last_pos = None
        self.pos = [0, 0]
        self.yaw = 0
        self.max_dis = 5
        self.sub_laser_upper = rospy.Subscriber(
            '/duckie_alpha/RL/scan', LaserScan, self.cb_laser, queue_size=1)

        # self.memory = self.scan_once()
        # for i in range(2):
        #     self.memory = np.vstack((self.memory, self.scan_once()))

    def get_initial_state(self, name, id):
        # start position
        state_msg = ModelState()
        state_msg.model_name = name
        state_msg.pose.position.x = INITIAL_STATES[id][0]
        state_msg.pose.position.y = INITIAL_STATES[id][1]
        state_msg.pose.position.z = INITIAL_STATES[id][2]
        quat = quaternion_from_euler(0, 0, np.random.uniform(-np.pi, np.pi))
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]
        return state_msg

    def cb_laser(self, msg):
        pass

    def set_max_dis(self, max_d):
        self.max_dis = max_d

    def scale_linear(self, n, bound):
        return (max(min(n, 1), 0))*bound

    def scale_angular(self, n, bound):
        return (max(min(n, 1), -1))*bound

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_physics()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]
        cmd_vel.angular.z = action[1]
        self.pub_twist.publish(cmd_vel)

        state = self.get_observation()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_physics()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        done = False
        info = {}

        ##################reward design##################
        angular_factor = (1-abs(action[1])) * 5
        reward = pow(2, angular_factor)
        reward *= action[0]

        min_dis = 100
        for i, dis in np.ndenumerate(state):
            if dis < min_dis:
                min_dis = dis
            if dis < 0.75:
                done = True

        if min_dis < 1:
            reward = -50 - 300*(1 - min_dis)

        max_r = (2**5)
        min_r = -50 - 300*(1 - 0.75)
        norm_reward = (reward-min_r)/(max_r-min_r)

        ##################reward design###################
        out_bound = False
        if np.amax(np.absolute(self.last_pos)) > 50:
            tmp_dis = self.total_dis
            self.reset()
            self.total_dis = tmp_dis
            out_bound = True

        return [self.pos], self.yaw, out_bound, norm_reward, done, info

    def reset(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            # self.reset_model(self.get_initial_state('duckie_alpha', 0))
            self.reset_world()
        except(rospy.ServiceException) as e:
            print e

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_physics()
        except(rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        self.reward = 0
        self.total_dis = 0
        self.pos = [0, 0]
        self.last_pos = None
        # self.memory = self.scan_once()
        # for i in range(2):
        #     self.memory = np.vstack((self.memory, self.scan_once()))

        state = self.get_observation()

        time.sleep(0.5)

        return state

    def scan_once(self):
        data1 = None
        while data1 is None:
            try:
                data1 = rospy.wait_for_message(
                    '/duckie_alpha/RL/scan', LaserScan, timeout=5)
            except:
                print('fail to receive message')

        laser1 = []
        for i, dis in enumerate(list(data1.ranges)):
            if dis > self.max_dis:
                dis = self.max_dis
            laser1.append(dis)
        return np.array(laser1)

    def get_observation(self):
        agent = ModelState()
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            agent = self.get_model('duckie_alpha', '')
        except (rospy.ServiceException) as e:
            print("/gazebo/get_model_state service call failed")

        new_pos = np.array(
            [agent.pose.position.x, agent.pose.position.y, agent.pose.position.z])
        self.pos = [agent.pose.position.x, agent.pose.position.y]

        quaternion = (agent.pose.orientation.x,
                      agent.pose.orientation.y,
                      agent.pose.orientation.z,
                      agent.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

        if self.last_pos is not None:
            self.total_dis += np.linalg.norm(new_pos-self.last_pos)
        self.last_pos = new_pos

        # self.memory = np.vstack((self.memory[1:, :], self.scan_once()))

        # state = np.reshape(self.memory,(-1))
        # state = self.memory
        state = self.scan_once()
        return state

    def close(self):
        self.unpause_physics()
        rospy.signal_shutdown('WTF')
