#!/usr/bin/env python3

import base64
import socket
import struct
import time 
import threading
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import logging 
import sys

# setup multi-thread logging


# setup socket 
TCP_IP = '172.17.0.2'  # faster than dedicated ip address https://docs.python.org/3/howto/sockets.html#ipc
TCP_PORT = 5010
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
MESSAGE = ''  # message to be sent to socket clients
NEW_IMAGE_RECIEVED = False  # logic to only send if image is new


# create socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(5)  # max 5 connections
    # TODO optimize muxing over multiple client sockets


# setup cv2 bridge
#       based on https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
bridge = CvBridge()
def image_callback(msg):
    global MESSAGE
    global NEW_IMAGE_RECIEVED

    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # encode image data
        img, buffer = cv2.imencode('.jpeg', frame)
        base64str = base64.b64encode(buffer)

        # setup message(s) to be sent over connection
        # prefix message with length of image encoded as an unsigned long long (8 bytes)
        MESSAGE = struct.pack("Q", len(base64str)) + base64str 
        NEW_IMAGE_RECIEVED = True
        print('Imgstr: ', MESSAGE[:50])



# setup and run ros in secondary thread
#       based on https://github.com/stoddabr/ros_flask/blob/master/src/main.py
def cleanup():
    global s
    s.close()  # close socket "gracefully"
    cv2.destroyAllWindows()   # only needed if ui is present but it shouldn't hurt

def init_ros(img_topic='/color/image_raw'):   # TODO pass topic from launch file arg or cmd
    rospy.init_node('frame_streamer', disable_signals=True)
    rospy.Subscriber(img_topic, Image, image_callback)   
    rospy.on_shutdown(cleanup) 

threading.Thread(target=init_ros).start()


# start listening
print('HOST IP: '+ host_ip)
print('HOST Port: '+ str(TCP_PORT))
print('Address: '+ host_ip+':'+str(TCP_PORT)+'/')
while True:
    # accept connections
    clientsocket, addr = s.accept()  # blocking
    print('New connection from '+ str(addr))
    if clientsocket:
        while True:
            time.sleep(0.05)  # avoid latency buildup over time
            if NEW_IMAGE_RECIEVED:
                # send image and prevent sending duplicates
                print('Sending:', MESSAGE[:20])
                clientsocket.sendall(MESSAGE)
                NEW_IMAGE_RECIEVED = False 

            # show image stream in pop-up window
            # cv2.imshow('Sending...', frame)
            # key = cv2.waitKey(10)
            # if key == 13:  # enter key
            #     break
