import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from time import sleep # Import the sleep function from the time module
import rospy,roslib

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbGPIO.setup(12, GPIO.OUT, initial=GPIO.LOW) # Set pin 8 to be an
GPIO.setup(12, GPIO.OUT, initial=GPIO.LOW)

def led_main():
    rospy.init_node('presence_led', anonymous=True)
    while not rospy.is_shutdown(): #accende e spegne il led 12
        GPIO.output(12, GPIO.HIGH)
        sleep(1)
        GPIO.output(12, GPIO.LOW)
        sleep(1)
        if rospy.is_shutdown(): #quando viene chiuso ros spegni il led
            GPIO.output(12, GPIO.LOW)

if __name__ == '__main__':
        led_main()
