import RPi.GPIO as GPIO
import time
 
GPIO.setmode(GPIO.BCM)
 
# Setup GPIO Pins
GPIO.setup(26, GPIO.OUT) #ENA
GPIO.setup(11, GPIO.OUT) #ENB
GPIO.setup(19, GPIO.OUT) #IN1
GPIO.setup(13, GPIO.OUT) #IN2
GPIO.setup(6, GPIO.OUT)  #IN3
GPIO.setup(5, GPIO.OUT)  #IN4
 
# Set PWM instance and their frequency
pwmR = GPIO.PWM(26, 100)
pwmL = GPIO.PWM(11, 100)
time.sleep(1)
# Start PWM with 50% Duty Cycle
pwmR.start(0)
pwmL.start(0)
GPIO.output(19, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)
GPIO.output(5, GPIO.HIGH)
time.sleep(1)
try:
    while(True):
        pwmR.ChangeDutyCycle(50)
        pwmL.ChangeDutyCycle(50)
       
 
except KeyboardInterrupt:
	pwmR.stop()
	pwmL.stop()
 
# Cleans the GPIO
GPIO.cleanup()
