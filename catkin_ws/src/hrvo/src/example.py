import sys
from RVO import RVO_update, reach, compute_V_des, reach
from vis import visualize_traj_dynamic
import cv2

#------------------------------
#define workspace model
ws_model = dict()
#robot radius
ws_model['robot_radius'] = 0.3
#circular obstacles, format [x,y,rad]
# no obstacles
ws_model['circular_obstacles'] = []
# with obstacles
#ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
#rectangular boundary, format [x,y,width/2,heigth/2]
ws_model['boundary'] = [] 

#------------------------------
#initialization for robot
# position of [x,y]
X = [[5,5],[5,0],[0,0],[0,5]]
# velocity of [vx,vy]
V = [[0,0] for i in xrange(len(X))]
# maximal velocity norm
V_max = [1 for i in xrange(len(X))]
# goal of [x,y]
goal = [[0,0],[0,5],[5,5],[5,0]]

#------------------------------
#simulation setup
# total simulation time (s)
total_time = 15
# simulation step
step = 0.01

#------------------------------
#simulation starts
t = 0
while t*step < total_time:
    # compute desired vel to goal
    V_des = compute_V_des(X, goal, V_max)
    # compute the optimal vel to avoid collision
    V = RVO_update(X, V_des, V, ws_model)
    # update position
    for i in xrange(len(X)):
        X[i][0] += V[i][0]*step
        X[i][1] += V[i][1]*step
    #----------------------------------------
    # visualization
    if t%10 == 0:
        #visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.pdf'%str(t/10))
        visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='data/snap%s.png'%str(t/10))
        print ("output result ",t/10)
        img = cv2.imread("data/snap%s.png"%str(t/10))
        heigth , width , _ = img.shape
        img = cv2.resize(img,(width/2,heigth/2))
        cv2.imshow("hrvo_demo",img)
        cv2.waitKey(1)
    t += 1
    
