import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIGR = 23
ECHOR = 24
TRIGL = 25
ECHOL = 8

GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)

GPIO.output(TRIGR, GPIO.LOW)
GPIO.output(TRIGL, GPIO.LOW)

time.sleep(1)


def get_distance():

    GPIO.output(TRIGR, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIGR, GPIO.LOW)
     
    # save StartTime
    while GPIO.input(ECHOR) == 0:
        startTimeR = time.time()
         
    # save time of arrival
    while GPIO.input(ECHOR) == 1:
        stopTimeR = time.time()

    GPIO.output(TRIGL, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIGL, GPIO.LOW)

    # save StartTime
    while GPIO.input(ECHOL) == 0:
        startTimeL = time.time()
         
    # save time of arrival
    while GPIO.input(ECHOL) == 1:
        stopTimeL = time.time()

    # time difference between start and arrival
    timeElapsedR = stopTimeR - startTimeR
    timeElapsedL = stopTimeL - startTimeL
    
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distanceR = (timeElapsedR * 34300) / 2
    distanceL = (timeElapsedL * 34300) / 2

    return (distanceL, distanceR)

try:
    while True:
        distL, distR = get_distance()
        print ("Measured Distance L= %.1f cm, Measured Distance R= %.1f cm" % (distL, distR))
        time.sleep(0.1)

    # Reset by pressing CTRL + C
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
