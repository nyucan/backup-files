import io
import socket
import struct
import time
import picamera
import RPi.GPIO as GPIO

class SplitFrames(object):
    def __init__(self, connection):
        self.connection = connection
        self.stream = io.BytesIO()
        self.count = 0

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # Start of new frame; send the old one's length
            # then the data
            size = self.stream.tell()
            if size > 0:
                self.connection.write(struct.pack('<L', size))
                self.connection.flush()
                self.stream.seek(0)
                self.connection.write(self.stream.read(size))
                self.count += 1
                self.stream.seek(0)
        self.stream.write(buf)

GPIO.setmode(GPIO.BCM)

# Setup GPIO Pins
GPIO.setup(26, GPIO.OUT) #ENA
GPIO.setup(11, GPIO.OUT) #ENB
GPIO.setup(19, GPIO.OUT) #IN1
GPIO.setup(13, GPIO.OUT) #IN2
GPIO.setup(6, GPIO.OUT)  #IN3
GPIO.setup(5, GPIO.OUT)  #IN4


client_socket = socket.socket()
client_socket.connect(('172.16.42.47', 8000))
connection = client_socket.makefile('wb')

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
    output = SplitFrames(connection)
    with picamera.PiCamera(resolution='VGA', framerate=30) as camera:
        time.sleep(2)

        
        camera.start_recording(output, format='mjpeg')
        
        try:
            while True:
                pwmR.ChangeDutyCycle(75)
                pwmL.ChangeDutyCycle(75)
                
        except KeyboardInterrupt:
            pwmR.stop()
            pwmL.stop()
        
        camera.stop_recording()
        # Write the terminating 0-length to the connection to let the
        # server know we're done
        connection.write(struct.pack('<L', 0))

finally:
    GPIO.cleanup()
    connection.close()
    client_socket.close()
    

