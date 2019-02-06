import io
import socket
import struct
import time
import picamera


client_socket = socket.socket()
client_socket.connect(('192.168.43.14', 8002))


connection = client_socket.makefile('wb')
try:
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    # Start a preview and let the camera warm up for 2 seconds
    camera.start_preview()
    time.sleep(2)


    start = time.time()
    stream = io.BytesIO()

    for foo in camera.capture_continuous(stream, 'jpeg', use_video_port = True):
        
        connection.write(struct.pack('<L', stream.tell()))
        connection.flush()
        
        stream.seek(0)
        connection.write(stream.read())
        # If we've been capturing for more than 30 seconds, quit
        if time.time() - start > 30:
            break
        # Reset the stream for the next capture
        stream.seek(0)
        stream.truncate()
    # Write a length of zero to the stream to signal we're done
    connection.write(struct.pack('<L', 0))
finally:
    connection.close()
    client_socket.close()
