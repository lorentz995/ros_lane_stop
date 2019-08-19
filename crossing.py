#!/usr/bin/env python
import rospy,sys,cv2,numpy,roslib
from geometry_msgs.msg import Twist
import traceback
from master_node.msg import *
from master_node.srv import *
import threading
from motor_stop import *

lock = True
jump = False
id_node = "stop"
positive_answ = 1


twistmessage = Twist()
followmessage = Follow()
followmessage.id = id_node

pub = rospy.Publisher('follow_topic', Follow, queue_size=1)

request_lock_service = rospy.ServiceProxy('request_lock',RequestLockService)
release_lock_service = rospy.ServiceProxy('release_lock',ReleaseLockService)

def requestCross(dec):
    global id_node, lock, jump
    if lock:
        cross(dec)
    elif jump:
        jump = False
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            cross(dec)
        else:
            msg_shared = rospy.wait_for_message("/lock_shared", Lock)
            checkMessage(msg_shared)

def releaseCross():
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


def turn_right():
    print("turn right")
    #rospy.init_node('ros_joy_controller', anonymous=True)
    twistmessage.linear.x=0
    twistmessage.linear.y=0
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(1.0, turn_right2)
    timer.start()

def turn_right2():
    print("turn right")
    #rospy.init_node('ros_joy_controller', anonymous=True)
    twistmessage.linear.x=100
    twistmessage.linear.y=100
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(1.1, right_rotation)
    timer.start()

def center():
    print("center")
    twistmessage.linear.x=100
    twistmessage.linear.y=100
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(1.0, center2)
    timer.start()
    #rospy.init_node('ros_joy_controller', anonymous=True)

def center2():
    twistmessage.linear.x=100
    twistmessage.linear.y=100
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(2.0,go_straight)
    timer.start()

def turn_left():
    print("turn left")
    #rospy.init_node('ros_joy_controller', anonymous=True)
    twistmessage.linear.x=0
    twistmessage.linear.y=0
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(1.0, turn_left2)
    timer.start()

def turn_left2():
    print("turn left")

    twistmessage.linear.x=100
    twistmessage.linear.y=100
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(0.5, left_rotation)
    timer.start()

def right_rotation(): # Ruota il robot di 90 gradi a destra
    print("right rotation")
    twistmessage = Twist()
    twistmessage.linear.x=80
    twistmessage.linear.y=-80
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(0.65, go_straight)
    timer.start()

def left_rotation(): # Ruota il robot di 90 gradi a sinistra
    print("left rotation")
    twistmessage = Twist()
    twistmessage.linear.x=30
    twistmessage.linear.y=80
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    timer = threading.Timer(5.0, go_straight)
    timer.start()

def go_straight():
    twistmessage.linear.x=0
    twistmessage.linear.y=0
    followmessage.twist = twistmessage
    pub.publish(followmessage)
    print("go straight")
    print("Rilasciando l'ACK")
    releaseCross()

def cross(dec):
    if dec == 'right':
        try:
            turn_right()
        except rospy.ROSInterruptException:
            pass

    elif dec == 'center':
        try:
            center()
        except rospy.ROSInterruptException:
            pass

    elif dec == 'left':
        try:
            turn_left()
        except rospy.ROSInterruptException:
            pass


