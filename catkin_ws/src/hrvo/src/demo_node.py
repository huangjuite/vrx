#!/usr/bin/env python

import rospy
from duckiepond_vehicle.msg import UsvDrive
from sensor_msgs.msg import NavSatFix,Imu
from nav_msgs.msg import Odometry
from RVO import RVO_update, reach, compute_V_des, reach
from PID import PID_control
from dynamic_reconfigure.server import Server
from control.cfg import ang_PIDConfig,dis_PIDConfig
import math
import tf


class BoatHRVO(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing" %self.node_name)

        #initiallize PID
        self.dis_pid = [PID_control("distance_control%d" % i) for i in range(4)]
        self.angu_pid = [PID_control("angular_control%d" % i) for i in range(4)]
        self.dis_server = Server(dis_PIDConfig,self.cb_dis_pid,"distance_control")
        self.ang_server = Server(ang_PIDConfig,self.cb_ang_pid,"angular_control")
        for i in range(4):
            self.dis_pid[i].setSampleTime(0.1)
            self.angu_pid[i].setSampleTime(0.1)
            self.dis_pid[i].SetPoint = 0
            self.angu_pid[i].SetPoint = 0

        #setup publisher
        self.pub_v1 = rospy.Publisher("/boat1/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat1/p3d_odom",Odometry,self.cb_boat1_odom,queue_size=1)

        self.pub_v2 = rospy.Publisher("/boat2/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat2/p3d_odom",Odometry,self.cb_boat2_odom,queue_size=1)

        self.pub_v3 = rospy.Publisher("/boat3/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat3/p3d_odom",Odometry,self.cb_boat3_odom,queue_size=1)

        self.pub_v4 = rospy.Publisher("/boat4/cmd_drive",UsvDrive,queue_size=1)
        self.sub_p3d1 = rospy.Subscriber("/boat4/p3d_odom",Odometry,self.cb_boat4_odom,queue_size=1)

        #initiallize boat status
        self.boat_odom = [Odometry() for i in range(4)]
        self.cmd_drive = [UsvDrive() for i in range(4)]
        self.yaw = [0 for i in range(4)]


        #initiallize HRVO environment
        self.ws_model = dict()
        #robot radius
        self.ws_model['robot_radius'] = 3
        self.ws_model['circular_obstacles'] = []
        #rectangular boundary, format [x,y,width/2,heigth/2]
        self.ws_model['boundary'] = [] 



        self.pin1 = [7.5,7.5]
        self.pin2 = [-7.5,7.5]
        self.pin3 = [-7.5,-7.5]
        self.pin4 = [7.5,-7.5]
        self.position = []
        self.goal = [self.pin3,self.pin4,self.pin1,self.pin2]
        #print(self.position)
        #print(self.goal)
        self.velocity = [[0,0] for i in range(4)]
        self.velocity_detect = [[0,0] for i in range(4)]
        self.v_max = [1 for i in range(4)]

        #timer
        self.timer = rospy.Timer(rospy.Duration(0.2),self.cb_hrvo)


    def cb_hrvo(self,event):
        self.update_all()
        v_des = compute_V_des(self.position,self.goal,self.v_max)
        self.velocity = RVO_update(self.position,v_des,self.velocity_detect,self.ws_model)
        #print("position",self.position)
        #print("velocity",self.velocity)
        info = [[0,0] for i in range(4)]
        for i in range(4):
            dis , angle = self.process_ang_dis(self.velocity[i][0],self.velocity[i][1],self.yaw[i])
            #self.dis_pid[i].update(dis)
            self.angu_pid[i].update(angle)
            #dis_out = max(min(self.dis_pid[i].output,1),-1) * -1
            dis_out = max(min(dis,1),-1)
            ang_out = max(min(self.angu_pid[i].output,1),-1)
            #print(i,dis_out,ang_out)
            self.cmd_drive[i] = self.control_cmd_drive(dis_out, ang_out)
            info[i][0] = self.cmd_drive[i].left
            info[i][1] = self.cmd_drive[i].right

        #print(info)
        self.pub_v1.publish(self.cmd_drive[0])
        self.pub_v2.publish(self.cmd_drive[1])
        self.pub_v3.publish(self.cmd_drive[2])
        self.pub_v4.publish(self.cmd_drive[3])

    def control_cmd_drive(self,dis,angle):
        cmd = UsvDrive()
        cmd.left = max(min(dis - angle,1),-1)
        cmd.right = max(min(dis + angle,1),-1)
        return cmd

    def process_ang_dis(self,vx,vy,yaw):
        dest_yaw = math.atan2(vy,vx)
        if dest_yaw > yaw:
            right_yaw = dest_yaw-yaw
            left_yaw = (2*math.pi-right_yaw)*-1
        else:
            left_yaw = dest_yaw-yaw
            right_yaw = (2*math.pi-(left_yaw*-1))
        #-1 < angle < 1 , find close side to turn
        angle = left_yaw if abs(left_yaw) < abs(right_yaw) else right_yaw
        angle = angle/math.pi
        dis = (((1 - abs(angle))-0.5) * 2) * pow(vx*vx + vy*vy,0.5)

        dis = max(min(dis,1),-1)
        angle = max(min(angle,1),0)
        return dis , angle

    def update_all(self):
        self.position = []
        for i in range(4):
            #update position
            pos = [self.boat_odom[i].pose.pose.position.x,self.boat_odom[i].pose.pose.position.y]
            self.position.append(pos)

            #update orientation
            quaternion = (self.boat_odom[i].pose.pose.orientation.x,
                        self.boat_odom[i].pose.pose.orientation.y,
                        self.boat_odom[i].pose.pose.orientation.z,
                        self.boat_odom[i].pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.yaw[i] = euler[2]
            
            #update velocity
            self.velocity_detect[i] = [self.boat_odom[i].twist.twist.linear.x,
                                self.boat_odom[i].twist.twist.linear.y]
            #print("\nboat%d" % i)
            #print(self.position[i])
            #print(self.yaw[i])
            #print(self.velocity[i])

    def cb_boat1_odom(self,msg):
        self.boat_odom[0] = msg

    def cb_boat2_odom(self,msg):
        self.boat_odom[1] = msg
    
    def cb_boat3_odom(self,msg):
        self.boat_odom[2] = msg

    def cb_boat4_odom(self,msg):
        self.boat_odom[3] = msg

    def cb_dis_pid(self,config,level):
        print("distance: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.dis_pid[i].setKp(Kp)
            self.dis_pid[i].setKi(Ki)
            self.dis_pid[i].setKd(Kd)
        return config

    def cb_ang_pid(self,config,level):
        print("angular: [Kp]: {Kp}   [Ki]: {Ki}   [Kd]: {Kd}\n".format(**config))
        Kp = float("{Kp}".format(**config))
        Ki = float("{Ki}".format(**config))
        Kd = float("{Kd}".format(**config))
        for i in range(4):
            self.angu_pid[i].setKp(Kp)
            self.angu_pid[i].setKi(Ki)
            self.angu_pid[i].setKd(Kd)
        return config


if __name__ == "__main__":
    rospy.init_node("BoatHRVO")
    boatHRVO = BoatHRVO()
    rospy.spin()

