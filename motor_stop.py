#!/usr/bin/env python
import rospy,sys,cv2,numpy,roslib
from geometry_msgs.msg import Twist
import traceback
from master_node.msg import *
from master_node.srv import *
import threading
from led import *

conta_stop = 0
one_time = False
id_node = "stop"
positive_answ = 1

twistmessage = Twist()
followmessage = Follow()
followmessage.id = id_node
lock = False
jump = False

pub = rospy.Publisher('follow_topic', Follow, queue_size=1)
request_lock_service = rospy.ServiceProxy('request_lock',RequestLockService)
release_lock_service = rospy.ServiceProxy('release_lock',ReleaseLockService)
stop_service = rospy.ServiceProxy('stop',StopService)

def requestLock():
    global id_node, lock, jump
    if lock:
        stop()
    elif jump:
        jump = False
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            stop()
        else:
            msg_shared = rospy.wait_for_message("/lock_shared", Lock)
            checkMessage(msg_shared)


def releaseLock():
    global id_node, lock
    resp = release_lock_service(id_node)
    lock = False
    print(resp)

def checkMessage(data):
    global id_node, lock
    if data.id == id_node:
        if data.msg == 1:
            lock = True
        else:
            lock = False
    else:
        msg_shared = rospy.wait_for_message("/lock_shared", Lock)
        checkMessage(msg_shared)

def stop():
    global conta_stop, one_time
    conta_stop += 1
    if conta_stop < 40:
        print("STOP")
        twistmessage.linear.x=0
        twistmessage.linear.y=0
        print(twistmessage)
        followmessage.twist = twistmessage
        pub.publish(followmessage)
        stop_service(0)
        if one_time == False:
            #video_filter()
            vid = threading.Timer(2.0, video_filter)
            vid.start()
            #rel = threading.Timer(6.0, requestLock)
            #rel.start()
            one_time = True



    '''else:
        twistmessage.linear.x=80
        twistmessage.linear.y=80
        followmessage.twist = twistmessage
        pub.publish(followmessage)
        releaseLock()
        twistmessage.linear.x=0
        twistmessage.linear.y=0
        followmessage.twist = twistmessage
        pub.publish(followmessage)'''


def reset():
    '''global conta_stop, one_time
    conta_stop = 0
    one_time = False'''
    video_filter()

