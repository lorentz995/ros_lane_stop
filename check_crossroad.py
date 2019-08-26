#!/usr/bin/env python2.7

import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy
from random import randint
from std_msgs.msg import Int32
import numpy as np

contatore1 = 0
contatore2 = 0
contatore3 = 0
contatore4 = 0
stop = False
via = False
direction = False
svolta = rospy.Publisher("svolta",Int32,queue_size=1)


def funzione(data):
    global contatore1, contatore2, contatore3, contatore4, stop, dir, via, direction
    contatore1 = 0
    contatore2 = 0
    contatore3 = 0
    contatore4 = 0
    stop = False
    direction = False
    status = data.data
    if status == 1:
        via = True
    elif status == 0:
        via = False

def callback(imgMsg):
    #rospy.loginfo()
    if via:
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
    global v, direction, dir
    led_a = 0
    led_d = 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(hsv, (11, 11), 0)
    ret,mask = cv2.threshold(blurred,140,250,cv2.THRESH_TOZERO)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=4)

    lower_red = np.array([160,90,90])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(hsv1, lower_red, upper_red)
    lower_yellow = np.array([0,100,100])
    upper_yellow = np.array([40,255,255])
    mask2 = cv2.inRange(hsv1, lower_yellow, upper_yellow)
    _, contours1, hierarchy1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

   #se vede il marker rosso non si puo' svoltare a destra
    for contour1 in contours1:
        currentArea1 = cv2.contourArea(contour1)
        approx1 = cv2.approxPolyDP(contour1, 0.01*cv2.arcLength(contour1, True), True)
        (x,y),radius = cv2.minEnclosingCircle(contour1)
        center = (int(x),int(y))
        if currentArea1 < 300:
            if 70 < x < 150 and 110 < y < 180:
                if direction == False:
                    cv2.drawContours(frame, [approx1], 0, (0,255,0), 2)
                    dir = randint(1,2)
                    if dir == 1:
                        print('voglio andare dritto')
                    if dir == 2:
                        print('voglio svoltare a sinistra')
                    direction = True

    _, contours2, hierarchy2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #se vede il marker giallo e' un incrocio completo
    for contour2 in contours2:
        currentArea2 = cv2.contourArea(contour2)
        approx2 = cv2.approxPolyDP(contour2, 0.01*cv2.arcLength(contour2, True), True)
        (x,y),radius = cv2.minEnclosingCircle(contour2)
        center = (int(x),int(y))
        if currentArea2 < 300:
            if 70 < x < 150 and 110 < y < 180:
                if direction == False:
                    cv2.drawContours(frame, [approx2], 0, (0,255,0), 2)
                    dir = randint(0,2)
                    if dir == 0:
                        print('voglio svoltare a destra')
                    if dir == 1:
                        print('voglio andare dritto')
                    if dir == 2:
                        print('voglio svoltare a sinistra')
                    direction = True

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
    #cv2.imshow('mask1 led', mask1)
    #cv2.imshow('mask2 led', mask2)
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
                    svolta.publish(10)
                    stop = True
                else:
                    print('Stop destro occupato ---> aspetto')

        if d >= 1 and a == 1:   #se destra occupata e anteriore occupato con led di movimento spento
            contatore2 = contatore2 + 1
            if contatore2 >= 70:
                if dir == 0:
                    print('Stop destro occupato e Stop frontale occupato con led di movimento spento ---> svolto a destra')
                    svolta.publish(10)
                    stop = True
                else:
                    print('Stop destro occupato e Stop frontale occupato con led di movimento spento ---> aspetto')

        if d >= 1 and a == 2:    #se destra occupata e anteriore occupato con led di movimento acceso
            print('Stop destro occupato, Stop frontale occupato con led di movimento acceso ---> aspetto')

        if d == 0 and a == 1:  #se destra libera e anteriore occupato con led di movimento spento
            contatore3 = contatore3 + 1
            if contatore3 >= 70:
                if dir == 0:
                    print('Stop frontale occupato con led di movimento spento ---> svolto a destra')
                    svolta.publish(10)
                    stop = True

                if dir == 1:
                    print('Stop frontale occupato con led di movimento spento ---> proseguo dritto')
                    svolta.publish(11)
                    stop = True

                if dir == 2:
                    print('Stop frontale occupato con led di movimento spento ---> svolto a sinistra')
                    svolta.publish(12)
                    stop = True

        if d == 0 and a == 2:   #se destra libera e anteriore occupato con led di movimento acceso
                print('Stop frontale occupato con led di movimento acceso ---> aspetto')

        if d == 0 and a == 0:   #se incrocio libero
            contatore4 = contatore4 + 1
            if contatore4 >= 70:
                if dir == 0:
                    print('incrocio libero ---> svolto a destra')
                    svolta.publish(10)
                    stop = True

                if dir == 1:
                    print('incrocio libero ---> proseguo dritto')
                    svolta.publish(11)
                    stop = True

                if dir == 2:
                    print('incrocio libero ---> svolto a sinistra')
                    svolta.publish(12)
                    stop = True

def led_filter():
    global contatore1, contatore2, contatore3, contatore4, stop, dir
    contatore1 = 0
    contatore2 = 0
    contatore3 = 0
    contatore4 = 0
    stop = False
    
    rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, callback)
    #rospy.Subscriber("camera_image",CompressedImage, callback)
    rospy.Subscriber("/check", Int32, funzione)
    rospy.spin()

if __name__=='__main__':
    rospy.init_node('image_subscriber')
    led_filter()
