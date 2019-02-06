import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIGR = 23
ECHOR = 24

def dist_callback(channel):
    timeElapsedR = time.time() - startTimeR
    distanceR = (timeElapsedR * 34300) / 2
    print (distanceR)
    
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.output(TRIGR, GPIO.LOW)
GPIO.add_event_detect(ECHOR, GPIO.FALLING, callback = dist_callback)

time.sleep(1)


def get_distance():

    GPIO.output(TRIGR, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIGR, GPIO.LOW)
    
    global startTimeR
    startTimeR = time.time()
    

try:
    while True:
        get_distance()
##        print ("Measured Distance L= %.1f cm" % distL)
        

    # Reset by pressing CTRL + C
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
