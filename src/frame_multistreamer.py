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
import sys
import errno
import os 
import logging

# setup multi-threading
RUN_BACKGROUND_THREADS = True  # kill flag

def killAllThreads():
    """ set flag to stop all secondary threads """
    # set flag for threads to die piecefully
    RUN_BACKGROUND_THREADS = False  # kill threads

# setup global socket variables 
TCP_IP = '172.17.0.2'  # faster than dedicated ip address https://docs.python.org/3/howto/sockets.html#ipc
TCP_PORT = 5010
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
MESSAGE = ''  # message to be sent to socket clients
NEW_IMAGE_RECIEVED = {}  # logic to only send if image is new, key is "thread id" val is bool


# setup cv2 bridge
#       based on https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f
bridge = CvBridge()
def image_callback(msg):
    """ on new ros image encode and update message """
    global MESSAGE
    global NEW_IMAGE_RECIEVED

    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print('CvBridgeError', e)
    else:
        # encode image data
        img, buffer = cv2.imencode('.jpeg', frame)
        base64str = base64.b64encode(buffer)

        # setup message(s) to be sent over connection
        # prefix message with length of image encoded as an unsigned long long (8 bytes)
        MESSAGE = struct.pack("Q", len(base64str)) + base64str 
        NEW_IMAGE_RECIEVED = dict.fromkeys(NEW_IMAGE_RECIEVED, True)  # set all threads to send new img
        # print('Imgstr: ', MESSAGE[:50])



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


# setup and run multi-threaded socket
# create socket globals
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(5)  # max 5 connections
s.settimeout(0.1)  # prevent code from hanging on `s.accept()`


def on_new_client(clientsocket, addr):
    """ serves video to clientsocket, designed to run in secondary thread(s) """
    global MESSAGE
    global RUN_BACKGROUND_THREADS
    global NEW_IMAGE_RECIEVED

    tid = threading.get_ident()  # current thread id
    NEW_IMAGE_RECIEVED[tid] = False
    t_init = time.perf_counter()
    t_last_frame = time.perf_counter()

    while RUN_BACKGROUND_THREADS:
        time.sleep(0.01)  # avoid latency caused by tcp traffic
        if clientsocket and NEW_IMAGE_RECIEVED[tid]:
            # send image and prevent sending duplicates
            try:
                t_start_send = time.perf_counter()
                clientsocket.sendall(MESSAGE)
                t_fin_send = time.perf_counter()
                print(f'Client timer, since start, {t_fin_send - t_init}, since last frame, {t_fin_send-t_last_frame}, send time, {t_fin_send-t_start_send}')
                t_last_frame = t_fin_send

                NEW_IMAGE_RECIEVED[tid] = False 
            except socket.error as e:
                if e == errno.EPIPE:
                    logging.error("Detected remote disconnect")
                else:
                    logging.error("Unhandled socket send error:" + str(e))
                break # exit send loop
            except Exception as e:
                print("Unhandled error thrown while sending message:", e)
                break # exit send loop

    # after thread killed cleanup socket
    clientsocket.close()


def server_listen():
    """ main server listening thread, spawns threads as clients connect """
    # start listening
    # accept connections
    try:
        clientsocket, addr = s.accept()
    except Exception as e:
        # s.accept() timed out so try again
        # part of a hack to keep s.accept() from blocking thread
        if str(e) == 'timed out':
            return  # try again 
    if clientsocket:
        print('Init client at', addr)
        threading.Thread(target=on_new_client, args=(clientsocket, addr,)).start()


# print server info
print('HOST IP: '+ host_ip)
print('HOST Port: '+ str(TCP_PORT))
print('Address: '+ host_ip+':'+str(TCP_PORT)+'/')

# main script
try:
    while True:
        server_listen()
except KeyboardInterrupt:
    print('keyboard exit triggered')
    killAllThreads()
except Exception as e:
    print('error in main loop:', e)

# gracefully shutdown
s.close()
killAllThreads()
cv2.destroyAllWindows() 
rospy.signal_shutdown("server loop exited")

