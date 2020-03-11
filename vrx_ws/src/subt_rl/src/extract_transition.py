#! /usr/bin/env python
import rospy
from subt_msgs.msg import *
from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
import pickle
import numpy as np
import copy


class Extract():
    def __init__(self):
        
        self.ep_trans = []
        self.memory = []
        self.current_obser = None
        self.auto = False
        self.count = 0

        self.sub_sc_process = rospy.Subscriber(
            '/df02/RL/scan_mmwave', LaserScan, self.cb_sc, queue_size=1)
        self.sub_cmd = rospy.Subscriber(
            '/cmd_vel', Twist, self.cb_twist, queue_size=1)
        self.sub_joy = rospy.Subscriber('/df02/joy', Joy, self.cb_joy, queue_size=1)

    def cb_twist(self,msg):
        if self.auto:
            print(msg.linear.x)
            print(msg.angular.z)

            state = copy.deepcopy(self.current_obser)
            action = []
            action.append(msg.linear.x)
            action.append(msg.angular.z)

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
            transition = [state, action, norm_reward]
            self.ep_trans.append(transition)
        else:
            if len(self.ep_trans) != 0:
                self.memory.append(self.ep_trans)
                self.ep_trans = []
                
    
    def cb_sc(self,msg):
        self.current_obser = msg.ranges
    
    def cb_joy(self,joy_msg):
        start_button = 7
        back_button = 6
        # Start button
        if (joy_msg.buttons[start_button] == 1) and not self.auto:
            self.auto = True
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1 and self.auto:
            self.auto = False
            rospy.loginfo('go manual')


    def on_shutdown(self):
        if len(self.ep_trans)!=0:
            self.memory.append(self.ep_trans)
        m = np.array(self.memory)
        print "saving", m.shape
        data = open('0204_df.pkl', 'wb')
        pickle.dump(self.memory, data)
        data.close()





if __name__ == "__main__":
    rospy.init_node("extracter")
    extr = Extract()
    rospy.on_shutdown(extr.on_shutdown)
    rospy.spin()
