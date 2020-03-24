#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Teleop(object):
    def __init__(self):
        self.auto = True
        self.turn_ratiao = 0.2
        self.pub_left_front = rospy.Publisher(
            "thrusters/left_front_thrust_cmd", Float32, queue_size=1)
        self.pub_right_front = rospy.Publisher(
            "thrusters/right_front_thrust_cmd", Float32, queue_size=1)
        self.pub_left_rear = rospy.Publisher(
            "thrusters/left_rear_thrust_cmd", Float32, queue_size=1)
        self.pub_right_rear = rospy.Publisher(
            "thrusters/right_rear_thrust_cmd", Float32, queue_size=1)

        sub_joy = rospy.Subscriber("/joy", Joy, self.cb_joy, queue_size=1)
        sub_cmd = rospy.Subscriber(
            "cmd_vel", Twist, self.cb_cmd, queue_size=1)

    def cb_cmd(self, msg):
        if self.auto:
            self.pub_right_front.publish(
                msg.linear.x + msg.angular.z)
            self.pub_right_rear.publish(
                (msg.linear.x + msg.angular.z)*0.7)

            self.pub_left_front.publish(
                msg.linear.x - msg.angular.z)
            self.pub_left_rear.publish(
                (msg.linear.x - msg.angular.z)*0.7)

    def cb_joy(self, joy_msg):
        # MODE X
        start_button = 7
        back_button = 6
        Logitech = 8
        RB = 5
        LB = 4
        A = 0
        X = 2

        # Start button
        if (joy_msg.buttons[start_button] == 1) and not self.auto:
            self.auto = True
            rospy.loginfo('go auto')
        elif joy_msg.buttons[back_button] == 1 and self.auto:
            self.auto = False
            rospy.loginfo('go manual')

        if not self.auto:
            self.pub_right_front.publish(
                joy_msg.axes[1] + joy_msg.axes[3])
            self.pub_right_rear.publish(
                (joy_msg.axes[1] + joy_msg.axes[3])*0.7)

            self.pub_left_front.publish(
                joy_msg.axes[1] - joy_msg.axes[3])
            self.pub_left_rear.publish(
                (joy_msg.axes[1] - joy_msg.axes[3])*0.7)


if __name__ == "__main__":
    rospy.init_node("teleop")
    teleop = Teleop()
    rospy.spin()
