#!/usr/bin/env python
import rospy,sys,cv2,numpy,roslib
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from master_node.msg import *
from master_node.srv import *
import numpy as np
import cv2
import threading
from geometry_msgs.msg import Twist
import traceback
from motor_stop import *

font = cv2.FONT_HERSHEY_COMPLEX
alt = False

#impostare il nome del nodo coerente con quello del master
id_node = "aruco" # scrivi (aruco, lane, stop, joy)... le priorita sono in ordine crescente
#impostare la risposta positiva coerente con quella del master
positive_answ = 1

twistmessage = Twist()
followmessage = Follow()
followmessage.id = id_node
lock = False
jump = False

pub = rospy.Publisher('follow_topic', Follow, queue_size=1) # publish on follow_topic
request_lock_service = rospy.ServiceProxy('request_lock',RequestLockService)
release_lock_service = rospy.ServiceProxy('release_lock',ReleaseLockService)
stop_service = rospy.ServiceProxy('stop',StopService)

def requestLock(data):
    global id_node, lock, jump
    if lock:
        frame_filter(data)
    elif jump:
        jump = False
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            frame_filter(data)
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




def frame_filter(imgMsg):
    global alt, id_node
    bridge = CvBridge()
    frame = bridge.compressed_imgmsg_to_cv2(imgMsg, "bgr8")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.waitKey(1)
    #Apply a red mask for stop detection
    #uncomment this code for use trackbars
    '''cv2.namedWindow("Trackbars")
    cv2.createTrackbar("L-H", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U-H", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)
    l_h = cv2.getTrackbarPos("L-H", "Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "Trackbars")
    lower_red = np.array([l_h,l_s,l_v])
    upper_red = np.array([u_h,u_s,u_v])'''
    #comment the next two records while trackbar is uncomment
    lower_red = np.array([160,90,90])
    upper_red = np.array([180,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    #for filter image
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask,kernel)
    #cv2.imshow("Mask",mask) #uncomment for view the mask filtering
    cv2.waitKey(1)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maxArea = 0
    bestContour = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        if area > maxArea: #Calculate max rectangular
            bestContour = approx
            maxArea = area
        cv2.drawContours(frame, [approx], 0, (0,0,255),1)
        if bestContour is not None:
            x,y,z,t = cv2.boundingRect(bestContour)
            cv2.rectangle(frame,(x,y),(x+z,y+t),(0,255,0),2)
            if area > 4000:
                cv2.putText(frame, "STOP DETECTION",(x,y), font, 1, (0,0,255))
                if y > 250:
                    try:
                        mainfunction2()
                    except Exception:
                        traceback.print_exc()





    cv2.imshow("Frame",frame)

    '''print("settando a 0 i motori")
    twistmessage.linear.x=0
    twistmessage.linear.y=0
    print(twistmessage)
    followmessage.twist = twistmessage
    pub.publish(followmessage)

    stop_service(int(0))'''

    #rospy.signal_shutdown("Stop")

def main_funcion():
    rospy.init_node('image_subscriber',anonymous=True)
    rospy.Subscriber("lock_shared",Lock,checkMessage)
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, requestLock)
    #rospy.Subscriber("/camera_image", CompressedImage, frame_filter)

    #Release on shutdown

    rospy.on_shutdown(releaseLock)
    rospy.spin()

if __name__=='__main__':
    main_funcion()
