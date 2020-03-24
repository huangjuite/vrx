#! /usr/bin/env python
import rospy
import tensorflow as tf
from sensor_msgs.msg import LaserScan, Joy, Imu
from rl_navigation.srv import pause, start, stop
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import os
from PID import PID_control
import time


class RunRDPGModel(object):
    def __init__(self):
        self.sim = rospy.get_param('~sim', False)
        self.auto = 0
        self.max_dis = 5

        model = rospy.get_param('~model', 'subt_rl495.ckpt-1037')
        output_name = rospy.get_param('~output_name','Actor/eval/action_out:0')
        input_name = rospy.get_param('~input_name','s:0')
        state_name0 = 'Actor/eval/LSTMCellZeroState/zeros:0'
        state_name1 = 'Actor/eval/LSTMCellZeroState/zeros_1:0'
        out_name0 = 'Actor/eval/rnn/while/Exit_3:0'
        out_name1 = 'Actor/eval/rnn/while/Exit_4:0'
        
        my_dir = os.path.abspath(os.path.dirname(__file__))
        input_checkpoint = os.path.join(my_dir, "../model/rdpg/" + model)

        saver = tf.train.import_meta_graph(
        input_checkpoint + '.meta', clear_devices=True)
        graph = tf.get_default_graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=config)
        saver.restore(self.sess, input_checkpoint)

        ##########  get tensor ##########
        self.input_s = graph.get_tensor_by_name(input_name)
        self.output_a = graph.get_tensor_by_name(output_name)

        lstm_state0 = graph.get_tensor_by_name(state_name0)
        lstm_state1 = graph.get_tensor_by_name(state_name1)
        self.lstm_state_in = (lstm_state0, lstm_state1)
        
        out_state0 = graph.get_tensor_by_name(out_name0)
        out_state1 = graph.get_tensor_by_name(out_name1)
        self.lstm_state_out = (out_name0, out_name1)

        self.hidden_state = (np.zeros([1,128]),np.zeros([1,128]))
        ##########  get tensor ##########

        self.action_bound = {'linear': 0.3, 'angular': 0.4}
        self.count = 0
        self.scan = None
        self.goal = 0
        self.heading = 0
        self.cbc = 0
        self.estop = False

        pause_srv = rospy.Service('RL/pause', pause, self.pause)
        stop_stv = rospy.Service('RL/stop',stop,self.stop_rl)
        start_srv = rospy.Service('RL/start', start, self.start_rl)
        self.sub_estop = rospy.Subscriber('e_stop',Bool,self.cb_estop,queue_size=1)
        self.sub_sc_process = rospy.Subscriber(
            'RL/scan', LaserScan, self.cb_sc, queue_size=1)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.cb_joy, queue_size=1)
        pub_name = 'cmd_vel'
        self.pub_twist = rospy.Publisher(pub_name, Twist, queue_size=1)

        time.sleep(1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_pub)
        self.pause_timer = None
        rospy.loginfo('press start')

    def cb_estop(self,msg):
        self.estop = msg.data

    def stop_rl(self,req):
        if self.auto == 1:
            response = "stop navigation"
            rospy.loginfo(response)
            self.auto = 0
            return response
        else:
            response = "joy stick mode"
            rospy.loginfo(response)
            return response

    def start_rl(self,req):
        if self.auto == 0:
            response = "start navigation"
            rospy.loginfo(response)
            self.auto = 1
            return response
        else:
            response = "running"
            rospy.loginfo(response)
            return response
    
    def pause(self,req):
        if self.auto == 1:
            response = "pause for 10 second"
            rospy.loginfo(response)
            self.pause_timer = rospy.Timer(rospy.Duration(10), self.resume_navigation)
            self.auto = 0
            return response
        else:
            response = "joy stick mode"
            rospy.loginfo(response)
            return response

    def resume_navigation(self,event):
        rospy.loginfo("resume")
        self.auto = 1
        self.smooth_vel = 0.35
        self.pause_timer.shutdown()

    def cb_sc(self, msg):
        self.scan = np.array(msg.ranges)
        self.scan = np.clip(self.scan,0,self.max_dis)

    def scale(self, n, bound):
        return max(min(n, 1), -1)*bound

    def timer_pub(self, event):
        if self.scan is not None and self.auto != 0 and self.estop != True:
            if self.auto == 1:
                laser = self.scan[np.newaxis, :]  # batch size
                actions, self.hidden_state = self.sess.run([self.output_a, self.lstm_state_out], {
                                                self.input_s: laser, self.lstm_state_in: self.hidden_state})
                action_out = actions[0]
                
                cmd_vel = Twist()
                cmd_vel.linear.x = self.scale(
                    action_out[0], self.action_bound['linear'])
                cmd_vel.angular.z = self.scale(
                    action_out[1], self.action_bound['angular'])
                self.pub_twist.publish(cmd_vel)

                self.smooth_vel = self.smooth_vel*0.99 + cmd_vel.linear.x*0.01
                if self.smooth_vel <= 0.07:
                    self.smooth_vel = 0.35
                    self.auto = 2
                    self.count = 0
            elif self.auto == 2:
                cmd_vel = Twist()
                cmd_vel.linear.x = -0.15
                cmd_vel.angular.z = 0
                self.pub_twist.publish(cmd_vel)
                self.count += 1
                if self.count > 15:
                    self.auto = 3
                    self.count = 0
            elif self.auto == 3:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = -0.2
                self.pub_twist.publish(cmd_vel)
                self.count += 1
                if self.count > 60:
                    self.auto = 1
                    self.count = 0
        
            


    def on_shutdown(self):
        self.sess.close()

    def cb_joy(self, joy_msg):
        # MODE X
        start_button = 7
        back_button = 6
        # Start button
        if (joy_msg.buttons[start_button] == 1) and not self.auto:
            self.auto = 1
            self.smooth_vel = 0.35
            self.hidden_state = (np.zeros([1,128]),np.zeros([1,128]))
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1 and self.auto:
            self.auto = 0
            rospy.loginfo('go manual')


if __name__ == '__main__':
    rospy.init_node('rl_rdpg')
    runmodel = RunRDPGModel()
    rospy.on_shutdown(runmodel.on_shutdown)
    rospy.spin()
