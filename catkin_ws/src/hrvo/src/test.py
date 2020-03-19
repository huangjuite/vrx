import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
import numpy as np

last_data = ""
started = False
pub = rospy.Publisher('/status', String, queue_size=1000) 

def callback(data):
    print "New message received"
    global started, last_data
    last_data = data
    if (not started):
        started = True

def timer_callback(event):
    global started, pub, last_data
    print "Last message published"


def listener():

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber('control_c', String, callback)
    timer = rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.spin()    
    timer.shutdown()

if __name__ == '__main__':
    print "Running"
    listener()