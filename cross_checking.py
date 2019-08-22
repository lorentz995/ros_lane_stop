#!/usr/bin/env python2.7

import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
from random import randint
from lane_stop_and_crossing import *

contatore1 = 0
contatore2 = 0
contatore3 = 0
contatore4 = 0
stop = False

def callback(imgMsg):
    #rospy.loginfo()
    try:
        bridge = CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(imgMsg)#, "bgr8")
        detection(image)

    except rospy.ROSInterruptException:
        print ("errore")
        pass

'''Controlla se ci sono led nelle zone di interesse. Se non ne trova lo stop di interesse e' libero, se ne trova 1 e'
occupato, se ne trova 2 e' occupato ma il duckiebot sta per partire.
N.B: 1 led ---> led di presenza, 2 led ---> led di presenza + led di intenzione di movimento'''
def detection(frame):
    led_a = 0
    led_d = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
    ret,mask = cv2.threshold(blurred,140,250,cv2.THRESH_TOZERO)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        currentArea = cv2.contourArea(contour)
        approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
        (x,y),radius = cv2.minEnclosingCircle(contour)
        center = (int(x),int(y))
        if currentArea < 400:
            #cerco quanti led ci sono nello stop frontale
            if 70 < y < 100 and 50 < x < 150:
                cv2.drawContours(frame, [approx], 0, (0,255,0), 2)
                led_a = led_a + 1

            #cerco quanti led ci sono nello stop di destra
            if 70 < y < 145 and 230 < x < 330:
                cv2.drawContours(frame, [approx], 0, (0,255,0), 2)
                led_d = led_d + 1

    check(led_a, led_d)

    cv2.imshow('frame', frame)
    #cv2.imshow('mask led', mask)
    cv2.waitKey(1)

#In base allo stato dell'incrocio restituisce la decisione su come attraversarlo
def check(a, d):
    global stop, contatore1, contatore2, contatore3, contatore4, dir
    if stop == False:
        if d >= 1 and a == 0: #se destra occupata e anteriore libero
            contatore1 = contatore1 + 1
            if contatore1 >= 70:
                if dir == 0:
                    print('Stop destro occupato ---> svolto a destra')
                    turn_right()
                    stop = True
                else:
                    print('Stop destro occupato ---> aspetto')

        if d >= 1 and a == 1:   #se destra occupata e anteriore occupato con led di movimento spento
            contatore2 = contatore2 + 1
            if contatore2 >= 70:
                if dir == 0:
                    print('Stop destro occupato e Stop frontale occupato con led di movimento spento ---> svolto a destra')
                    turn_right()
                    stop = True
                else:
                    print('Stop destro occupato e Stop frontale occupato con led di movimento spento ---> aspetto')

        if d >= 1 and a == 2:    #se destra occupata e anteriore occupato con led di movimento acceso
            print('Stop destra occupato, Stop frontale occupato con led di movimento acceso ---> aspetto')

        if d == 0 and a == 1:  #se destra libera e anteriore occupato con led di movimento spento
            contatore3 = contatore3 + 1
            if contatore3 >= 70:
                if dir == 0:
                    print('Stop frontale occupato con led di movimento spento ---> svolto a destra')
                    turn_right()
                    stop = True

                if dir == 1:
                    print('Stop frontale occupato con led di movimento spento ---> proseguo dritto')
                    go_straight()
                    stop = True

                if dir == 2:
                    print('Stop frontale occupato con led di movimento spento ---> svolto a sinistra')
                    turn_left()
                    stop = True

        if d == 0 and a == 2:   #se destra libera e anteriore occupato con led di movimento acceso
                print('Stop frontale occupato con led di movimento acceso ---> aspetto')

        if d == 0 and a == 0:   #se incrocio libero
            contatore4 = contatore4 + 1
            if contatore4 >= 70:
                if dir == 0:
                    print('incrocio libero ---> svolto a destra')
                    turn_right()
                    stop = True

                if dir == 1:
                    print('incrocio libero ---> proseguo dritto')
                    go_straight()
                    stop = True

                if dir == 2:
                    print('incrocio libero ---> svolto a sinistra')
                    turn_left()
                    stop = True

def led_filter():
    print("led filter")
    global contatore1, contatore2, contatore3, contatore4, stop, dir
    contatore1 = 0
    contatore2 = 0
    contatore3 = 0
    contatore4 = 0
    stop = False
    #estrae una direzione casuale
    dir = randint(0,2)

    if dir == 0:
        print('voglio svoltare a destra')
    if dir == 1:
        print('voglio andare dritto')
    if dir == 2:
        print('voglio svoltare a sinistra')
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, callback)
    #rospy.Subscriber("camera_image",CompressedImage, callback)
    rospy.spin()



if __name__=='__main__':
    rospy.init_node('image_subscriber')
    led_filter()
