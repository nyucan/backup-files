import io
import time
import threading
import picamera
from PIL import Image
import numpy as np
import cv2
import RPi.GPIO as GPIO

MARGIN_LEFT = 260
MARGIN_RIGHT = 380

'''---------------GPIO SECTION ----------'''
GPIO.setmode(GPIO.BCM)

TRIGR = 23
ECHOR = 24
TRIGL = 25
ECHOL = 8

ENA = 26
ENB = 11
IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5

GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)

GPIO.output(TRIGR, GPIO.LOW)
GPIO.output(TRIGL, GPIO.LOW)

time.sleep(1) #just small delay

#  Motor Pins
GPIO.setup(ENA, GPIO.OUT) #ENA
GPIO.setup(ENB, GPIO.OUT) #ENB
GPIO.setup(IN1, GPIO.OUT) #IN1
GPIO.setup(IN2, GPIO.OUT) #IN2
GPIO.setup(IN3, GPIO.OUT)  #IN3
GPIO.setup(IN4, GPIO.OUT)  #IN4

# PWM pin and Frequency
pwmR = GPIO.PWM(26, 100)
pwmL = GPIO.PWM(11, 100)

pwmR.start(0)
pwmL.start(0)

time.sleep(1)

'''--------------------------------------'''

# Set motor forwards
GPIO.output(19, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)
GPIO.output(5, GPIO.HIGH)
time.sleep(1)

distance = []

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

def absJudge(_abs):
    if _abs>250:
        return 250
    elif _abs<130:
        return 130
     
    return _abs
    
def runningADP(dist, prevDist):
    setPoint = 35
    setSpeed = 30
    distr = dist - setPoint
    speedr = (dist - prevDist) / 0.3
    ABS = 1.7992*distr + 4.1274*speedr
    
    ABS = absJudge(ABS)
    
    return ABS, speedr
    
class ImageProcessor(threading.Thread):
    def __init__(self, owner):
        super(ImageProcessor, self).__init__()
        self.stream = io.BytesIO()
        self.event = threading.Event()
        self.terminated = False
        self.owner = owner
        self.start()

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    distL, distR = get_distance()
                    distMin = min(distL, distR)
                   
                    if (distMin < 30):
                        pwmR.ChangeDutyCycle(0)
                        pwmL.ChangeDutyCycle(0)
                        distance.append(distMin)
                    else:    
                        self.stream.seek(0)
                        # Read the image and do some processing on it
                        image = Image.open(self.stream)
                        im = np.array(image)

                        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

                        circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100)
                        
                        if circles is not None:
                            # convert the (x, y) coordinates and radius of the circles to integers
                            circles = np.round(circles[0, :]).astype("int")
                                
                           # Control Algorithm here
                            '''
                            if distance:
                                speed_abs, speedr = runningADP(distMin, distance.pop())
                            else:
                                speed_abs, speedr = runningADP(distMin, 0)
                            print ("{:,.2f} {:,.2f} {:,.2f} {:,.2f}".format(distL, distR,  speedr,  speed_abs))
                            speed_abs = (int)((speed_abs * 100) / 255)
                            '''
                            if(circles[0][0] < MARGIN_LEFT):
                                print ('L')
                                #pwmR.ChangeDutyCycle(speed_abs)
                                #pwmL.ChangeDutyCycle(speed_abs - 30)
                            elif (circles[0][0] > MARGIN_LEFT and circles[0][0] < MARGIN_RIGHT):
                                print ('C')
                                #pwmR.ChangeDutyCycle(speed_abs)
                                #pwmL.ChangeDutyCycle(speed_abs)
                            elif (circles[0][0] > MARGIN_RIGHT):
                                print ('R')
                                #pwmR.ChangeDutyCycle(speed_abs -  30)
                                #pwmL.ChangeDutyCycle(speed_abs)
                            #distance.append(distMin)
                            

                        
                except KeyboardInterrupt:
                    print ("while in main loop")
                    pwmR.stop()
                    pwmL.stop()
                    GPIO.cleanup()
                    
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the available pool
                    with self.owner.lock:
                        self.owner.pool.append(self)

class ProcessOutput(object):
    def __init__(self):
        self.done = False
        # Construct a pool of 4 image processors along with a lock
        # to control access between threads
        self.lock = threading.Lock()
        self.pool = [ImageProcessor(self) for i in range(1)]
        self.processor = None

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame; set the current processor going and grab
            # a spare one
            if self.processor:
                self.processor.event.set()
            with self.lock:
                if self.pool:
                    self.processor = self.pool.pop()
                else:
                    # No processor's available, we'll have to skip
                    # this frame; you may want to print a warning
                    # here to see whether you hit this case
                    self.processor = None
        if self.processor:
            self.processor.stream.write(buf)

    def flush(self):
        # When told to flush (this indicates end of recording), shut
        # down in an orderly fashion. First, add the current processor
        # back to the pool
        if self.processor:
            with self.lock:
                self.pool.append(self.processor)
                self.processor = None
        # Now, empty the pool, joining each thread as we go
        while True:
            with self.lock:
                try:
                    proc = self.pool.pop()
                except IndexError:
                    pass # pool is empty
            proc.terminated = True
            proc.join()
try:
    with picamera.PiCamera(resolution='VGA') as camera:
        
        time.sleep(2)
        output = ProcessOutput()
        camera.start_recording(output, format='mjpeg')
        while not output.done:
            camera.wait_recording(1)
        camera.stop_recording()
except KeyboardInterrupt:
    pwmR.stop()
    pwmL.stop()
 
    # Cleans the GPIO
    GPIO.cleanup()

