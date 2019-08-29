import RPi.GPIO as GPIO
from time import sleep
import rospy,sys,cv2,roslib, time, threading
from std_msgs.msg import Int32

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbGPIO.setup(12, GPIO.OUT, initial=GPIO.LOW) # Set pin 8 to be an
GPIO.setup(40, GPIO.OUT, initial=GPIO.LOW)

def start():
	x=0
	while x<4:
		GPIO.output(40, GPIO.HIGH)
		sleep(2.5)
		GPIO.output(40, GPIO.LOW)
		sleep(0.5)
		x += 1

def main_function():
    rospy.init_node('movement_led',anonymous=True)
    rospy.Subscriber("/led", Int32, start)
    rospy.on_shutdown(GPIO.output(40, GPIO.LOW))
    rospy.spin()

if __name__=='__main__':
    main_function()
