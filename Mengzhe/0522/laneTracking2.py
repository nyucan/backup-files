'''
Only lane tracking with no car following
'''
import io
import time
import picamera
from PIL import Image
import numpy as np
import cv2
import RPi.GPIO as GPIO
from processImage import processImage
from serverClass import Server
import sys

MARGIN_LEFT = 260
MARGIN_RIGHT = 380
turningKL = 4.2 #f5or lane tracking, turning left
turningKR = 1 #for lane tracking, turning right


HOST = '0.0.0.0'
PORT = 5001
'''---------------GPIO SECTION ----------'''
GPIO.setmode(GPIO.BCM)


ENA = 26
ENB = 11
IN1 = 19
IN2 = 13
IN3 = 6
IN4 = 5

time.sleep(1) 

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
GPIO.output(19, GPIO.HIGH)
GPIO.output(13, GPIO.LOW)
GPIO.output(6, GPIO.HIGH)
GPIO.output(5, GPIO.LOW)
time.sleep(1)
print ('GPIO INITIALIZED')


distance = []
data = []
stream = io.BytesIO()


def absJudge(_abs):
    if _abs>220:
        return 220
    elif _abs<100:
        return 100
     
    return _abs
    
def runningADP(dist, prevDist):
    
    setPoint = 35
    setSpeed = 30
    distr = dist - setPoint
    speedr = (dist - prevDist) / 0.3
   
    #ABS = 0.3313*distr + 3.3963*speedr + 163.1503
    
    #ABS = absJudge(ABS)
    ABS = 110 
    
    return ABS, speedr

count = 0
#s = Server(HOST, PORT)
imgProc = processImage()
time.sleep(1)
firstStart = True
prevEdgeL = 0
lastEdge_L, lastEdge_R = 0, 0
lastEdgeTop_L, lastEdgeTop_R = 0, 0
try:
    with picamera.PiCamera(resolution='VGA') as camera:
            with io.BytesIO() as stream:
                    for frame in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
                        #aa = time.time()
                        stream.seek(0)
                        image = Image.open(stream)
                        im = np.array(image)
                            
                        key = cv2.waitKey(1) & 0xFF #necessary for cv2

                        redIMG = imgProc.redFilter(im) # find the red plate
                        
                        contours, hierarchy = cv2.findContours(redIMG, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        if (firstStart == True):
                            pwmR.ChangeDutyCycle(50)
                            pwmL.ChangeDutyCycle(35)
                            firstStart = False
                            
                        #if (len(contours)>0):
                        if (True):
                            
                            #x,y,w,h = imgProc.getRectangle(contours)
                            
                            #cv2.rectangle(redIMG, (x,y), (x+w,y+h), (255,0,0), 2) #for display purposes
                            #camDist = int(5400 / w)   #distance in cm
                            

                            #if (camDist < 30):
                            if(False):
                                pwmR.ChangeDutyCycle(0)
                                pwmL.ChangeDutyCycle(0)
                                
                            else:
                                camDist = 35
                                laneError  = 0
                                lines, imgTrim = imgProc.getLines(im)
                                if(lines is not None):
                                   # Filter lines above theta threshold; theta near pi/2
                                    lines = lines[0][(lines[0,:,1]<(np.pi/2-0.35))|(lines[0,:,1]>(np.pi/2+0.35))]
                                    if(len(lines) != 0):
                                        # egdgePoint gives the intersection of the detected line and the bottom line of the image
                                        edgePoints = np.array(list(map(lambda rho, theta: (rho - (imgTrim.shape[0] * np.sin(theta))) / np.cos(theta), lines[:,0], lines[:,1])))

                                        #if edgepoint shows on the right half of the image, then 'See R line' 
                                        if edgePoints[edgePoints > imgTrim.shape[1]/2].size != 0:
                                            #print("See R line")
                                            value_R = np.amin(edgePoints[edgePoints > imgTrim.shape[1]/2]) # find the point closest to the centerline
                                            ind_Line_R = np.where(edgePoints == value_R) # find the index of the line, which has the point closest to the centerline
                                            imgProc.drawLine(imgTrim, lines[ind_Line_R,0], lines[ind_Line_R,1]) # draw the line: rho, theta, definition is in processImage.py
                                            edgeR = imgProc.calculateEdgePoint(lines[ind_Line_R,0], lines[ind_Line_R,1], imgTrim.shape[0])# it's the same as value_R
                                            lastEdge_R = edgeR # logic for losing edgeR, save it for later
                                            edgeR_top = imgProc.calculateEdgePoint(lines[ind_Line_R,0], lines[ind_Line_R,1], 0.0)
                                            lastEdgeTop_R = edgeR_top

                                        if edgePoints[edgePoints < imgTrim.shape[1]/2].size != 0:
                                            #print("See L line")
                                            value_L = np.amax(edgePoints[edgePoints < imgTrim.shape[1]/2])
                                            ind_Line_L = np.where(edgePoints == value_L)
                                            imgProc.drawLine(imgTrim, lines[ind_Line_L,0], lines[ind_Line_L,1])
                                            edgeL = imgProc.calculateEdgePoint(lines[ind_Line_L,0], lines[ind_Line_L,1], imgTrim.shape[0])
                                            if abs(edgeR-edgeL)>30: lastEdge_L = edgeL
                                            edgeL_top = imgProc.calculateEdgePoint(lines[ind_Line_L,0], lines[ind_Line_L,1], 0.0)
                                            lastEdgeTop_L = edgeL_top
                                            
                                            
                                        lane_centre = int(abs(lastEdge_L+lastEdge_R)/2)
                                        imgProc.drawCircle(imgTrim, lane_centre, imgTrim.shape[0], 5, (0,0,255)) # plot the center of the detected edge points at bottom
                                        imgProc.drawCircle(imgTrim, imgTrim.shape[1]/2, imgTrim.shape[0], 10, (255,0,0)) #plot the center of the image
                                        roadOffset = imgTrim.shape[1] / 2 - lane_centre
                                        
                                        #We also calculate the  point of the centre of the lane reachable by the frame trimmed
                                        centre_top = int(abs(lastEdgeTop_L+lastEdgeTop_R)/2)
                                        imgProc.drawCircle(imgTrim, centre_top, 0, 5, (0,0,255))
                                        angle = np.arctan2(centre_top-imgTrim.shape[1]/2, imgTrim.shape[0])
                                        
                                        #print(angle, imgTrim.shape[0])
                                        
                                        #if roadOffset > 20: laneError = (turningKL * roadOffset - 0.0794) / 0.0053  #turn left
                                        #elif roadOffset < - 20: laneError = (turningKR * (-1) * roadOffset + 0.425) / 0.0111
                                        #elif roadOffset < - 20: laneError = (turningKR * (-1) * roadOffset - 0.0425) / 0.0111
                                        
                                        if roadOffset > 5: #turn left
                                            laneError = turningKL * roadOffset
                                        elif roadOffset < -5: # turn right
                                            laneError = turningKR * -1 * roadOffset
                                        
                                        if distance:
                                            speed_abs, speedr = runningADP(camDist, distance.pop())
                                        else:
                                            speed_abs, speedr = runningADP(camDist, 0)
                                        
                                        #print ("{} {} {:,.2f} {:,.2f}".format(count, camDist,  speedr,  speed_abs))
                                        

                                        speed_abs = (int)((speed_abs * 100) / 255)
                                        if (speed_abs - laneError < 0): laneError = speed_abs
         
                                        
                                        if(roadOffset > 4):
                                            ar, al = 0,0 
                                            if laneError > 65: laneError = 65

                                            if speed_abs + 0.95*laneError > 100 :
                                                pwmR.ChangeDutyCycle(90)
                                                ar = 90
                                            else:
                                                pwmR.ChangeDutyCycle(speed_abs+0.95*laneError)
                                                ar = speed_abs+0.95*laneError
                                            
                                            if speed_abs - laneError*0.5 < 0:
                                                pwmL.ChangeDutyCycle(0)
                                                al = 0
                                            else:
                                                pwmL.ChangeDutyCycle(speed_abs - laneError*0.5)
                                                al = speed_abs - laneError*0.5
                                            print ('L: {} {:,.2f} {:,.2f} {:,.2f}'.format(roadOffset, angle, ar, al))
                                            
                                        elif (roadOffset >= -4 and roadOffset <= 4):
                                           
                                            print ("C: {} {:,.2f} {:,.2f} {:,.2f}".format(roadOffset, angle, speed_abs - 10, speed_abs))
                                            #print ('C')
                                            speed_abs = 45
                                            pwmR.ChangeDutyCycle(speed_abs - 10)
                                            pwmL.ChangeDutyCycle(speed_abs)
                                            
                                        elif (roadOffset < -4):
                                            ar, al = 0,0 
                                            if laneError > 65: laneError = 65

                                            if speed_abs + 0.5*laneError > 100 :
                                                pwmL.ChangeDutyCycle(90)
                                                al = 90
                                            else:
                                                pwmL.ChangeDutyCycle(speed_abs+0.5*laneError)
                                                al = speed_abs + laneError*0.5
                                            
                                            if speed_abs - laneError*0.5 < 0:
                                                pwmR.ChangeDutyCycle(0)
                                                ar  = 0 
                                            else:
                                                pwmR.ChangeDutyCycle(speed_abs - laneError*0.5)
                                                ar = speed_abs - laneError*0.5
                                                
                                            print ("R: {} {:,.2f} {:,.2f} {:,.2f}".format(roadOffset, angle, ar, al))
                                        distance.append(camDist) #check indentation        
                                else:
                                    pwmR.ChangeDutyCycle(0)
                                    pwmL.ChangeDutyCycle(0)
                                    print ('No Lane!')
                                cv2.imshow('redIMG', imgTrim)
                        stream.seek(0)
                        stream.truncate() # remove all previous images
                        #data = s.getData()
                        #print (data)
                        count += 1
                        #print (time.time() - aa)
                                    
except KeyboardInterrupt:
    pwmR.stop()
    pwmL.stop()
    GPIO.cleanup()
    #s.closeSocket()
    
  
