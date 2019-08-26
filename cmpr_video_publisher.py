#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

def image_publisher():
    #Inizializzar il publisher
    pub = rospy.Publisher('camera_image', CompressedImage, queue_size=1)
    rospy.init_node('image_publisher', anonymous=True)

    bridge = CvBridge() #inizializza la classe CvBridge
    #Avvia fotocamera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 2) #Setta la fotocamera a 10 fps
    print("Avvio Webcam riuscito. CRTL + C per uscire")
    #rval, frame = cap.read()

    while not rospy.is_shutdown(): #Premi CTRL-C per uscire da terminale

        #cv2.imshow("Stream: ", frame)
        rval, frame = cap.read()
        resize = cv2.resize(frame,(410, 308))

        image_message = bridge.cv2_to_compressed_imgmsg(resize)#, encoding="bgr8")  #utilizza il cvbridge per convertire da cv2 a compressedimage

        pub.publish(image_message)  #pubblica sul topic
        #print("Ho inviato un fotogramma")

        #Gestione dell'uscita: premi Esc per uscire
        #key = cv2.waitKey(1)
        #if key == 27 or key == 1048603:
            #break
    #cv2.destroyWindow("preview")


if __name__ == '__main__':
    try:
        image_publisher()

    except rospy.ROSInterruptException:
        pass
