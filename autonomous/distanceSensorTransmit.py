import RPi.GPIO as GPIO
import time
import socket


GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, GPIO.LOW)
time.sleep(1)


PORT_NUMBER = 5002
IP = '172.16.45.236'

canSocket = socket.socket( socket.AF_INET, socket.SOCK_STREAM)
canSocket.connect((IP, PORT_NUMBER))


def get_distance():
    
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return str(distance)



try:
    while True:
        dist = get_distance()
        print ("Measured Distance = %s cm" % dist)
        canSocket.send(str.encode(dist))
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Measurement stopped by User")
    canSocket.close()
    GPIO.cleanup()

