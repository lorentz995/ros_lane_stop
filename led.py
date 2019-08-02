#!/usr/bin/env python2.7

import cv2
import numpy as np
import argparse
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy, sys
from random import randint
from crossing import *
from master_node.msg import *
from master_node.srv import *

contatore1 = 0
contatore2 = 0
contatore3 = 0
contatore4 = 0
stop = False

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

def requestLock(data):
    global id_node, lock, jump
    if lock:
        callback(data)
    elif jump:
        jump = False
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            callback(data)
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


def callback(imgMsg):
    #rospy.loginfo()
    try:
        bridge = CvBridge()
        originale = bridge.compressed_imgmsg_to_cv2(imgMsg)#, "bgr8")
        filtra(originale)

    except rospy.ROSInterruptException:
        print ("errore")
        pass


def filtra(frame):
    destra = False
    anteriore = False
    led_d = False
    led_a = False


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
    ret,mask = cv2.threshold(blurred,180,250,cv2.THRESH_TOZERO)

    lower_blue = np.array([26,77,48])
    upper_blue = np.array([141,255,255])

    mask1 = cv2.inRange(hsv1, lower_blue, upper_blue)

    element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    mask1 = cv2.erode(mask1,element, iterations=2)
    mask1 = cv2.dilate(mask1,element,iterations=4)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    _, contours1, hierarchy1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour1 in contours1:
        currentArea1 = cv2.contourArea(contour1)
        approx1 = cv2.approxPolyDP(contour1, 0.01*cv2.arcLength(contour1, True), True)
        (x,y),radius = cv2.minEnclosingCircle(contour1)
        center = (int(x),int(y))
        if currentArea1 < 600:

            #se trovo il marker a sinistra
            if 70 < y < 100 and 100 < x < 150:
                cv2.drawContours(frame, [approx1], 0, (0,255,0), 2)
                anteriore = True


            #se trovo il marker a destra
            if 70 < y < 145 and 265 < x < 335:
                cv2.drawContours(frame, [approx1], 0, (0,255,0), 2)
                destra = True



    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        currentArea = cv2.contourArea(contour)
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
        (x,y),radius = cv2.minEnclosingCircle(contour)
        center = (int(x),int(y))
        if currentArea < 300:

            #se trovo un led a sinistra
            if 70 < y < 100 and 100 < x < 170:
                cv2.drawContours(frame, [approx], 0, (0,255,0), 2)
                led_a = True


            #se trovo un led a destra
            #if 70 < y < 145 and 265 < x < 335:
                #cv2.drawContours(frame, [approx], 0, (0,255,0), 2)
                #led_d = True

    attraversamento(destra, anteriore, led_a)


    cv2.imshow('led', mask)
    cv2.imshow('marker blu', mask1)
    cv2.imshow('frame', frame)

    cv2.waitKey(1)


def attraversamento(d, a, l_a):
    global stop, contatore1, contatore2, contatore3, contatore4
    if stop == False:
        if d == True and a == False: #se destra occupata e anteriore libero
            contatore1 = contatore1 + 1
            if contatore1 >= 70:
                dir = randint(0,2)
                if dir == 0:
                    #decisione('right')
                    print('dx occupata, giro a dx')
                    stop = True
                else:
                    print('dx occupata, aspetto')



        if d == True and a == True and l_a == False:   #se destra occupata e anteriore occupato con led spento
            contatore2 = contatore2 + 1
            if contatore2 >= 70:
                dir = randint(0,2)
                if dir == 0:
                    requestLock('right')
                    print('dx occupata e anteriore occupato con led spento, giro a dx')
                    stop = True
                else:
                    print('dx occupata e anteriore occupato con led spento, aspetto')


        if d == True and a == True and l_a == True:    #se destra occupata e anteriore occupato con led acceso
            print('destra  e anteriore occupati e led acceso, quindi aspetto')

        if d == False and a == True and l_a == False:  #se destra libera e anteriore occupato con led spento
            contatore3 = contatore3 + 1
            if contatore3 >= 70:
                dir = randint(0,2)
                if dir == 0:
                    requestLock('right')
                    print('anteriore occupato e led spento, giro a destra')
                    stop = True

                if dir == 1:
                    requestLock('center')
                    print('anteriore occupato e led spento, vado dritto')
                    stop = True

                if dir == 2:
                    requestLock('left')
                    print('anteriore occupato e led spento, giro a sx')
                    stop = True

        if d == False and a == True and l_a == True:   #se destra libera e anteriore occupato con led acceso
                print('anteriore occupato e led acceso, quindi aspetto')

        if d == False and a == False:   #se incrocio libero
            contatore4 = contatore4 + 1
            if contatore4 >= 70:
                dir = randint(0,2)

                if dir == 0:
                    requestLock('right')
                    print('incrocio libero, giro a dx')
                    stop = True

                if dir == 1:
                    requestLock('center')
                    print('incrocio libero, vado dritto')
                    stop = True

                if dir == 2:
                    requestLock('left')
                    print('incrocio libero, vado a sx')
                    stop = True


def video_filter():
    #rospy.init_node('image_subscriber')
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, requestLock)
    #rospy.Subscriber("camera_image",CompressedImage, callback)
    rospy.on_shutdown(releaseLock)
    rospy.spin()


if __name__=='__main__':
    video_filter()
