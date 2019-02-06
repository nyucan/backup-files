import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(26, GPIO.OUT) #ENA 
GPIO.setup(19, GPIO.OUT) #IN1
GPIO.setup(13, GPIO.OUT) #IN2 
GPIO.setup(6, GPIO.OUT) #IN3 
GPIO.setup(5, GPIO.OUT) #IN4
GPIO.setup(11, GPIO.OUT) #ENB 


GPIO.output(13, GPIO.LOW)
GPIO.output(19, GPIO.LOW)
GPIO.output(26, GPIO.LOW)



GPIO.output(26, GPIO.HIGH)
GPIO.output(19, GPIO.HIGH)
GPIO.output(13, GPIO.LOW)
GPIO.output(6, GPIO.HIGH)
GPIO.output(5, GPIO.LOW)
GPIO.output(11, GPIO.HIGH)
try:
    time.sleep(1)
except KeyboardInterrupt:
   GPIO.cleanup() 

GPIO.cleanup()

