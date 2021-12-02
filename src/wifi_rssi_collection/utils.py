import os
import sys
import glob
import rospy
import time
import itertools
import datetime
import threading
import numpy as np, copy
from scapy.all import sniff
from rospy.msg import AnyMsg
from subprocess import Popen, PIPE
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def extractData(dictionary_list, key, delimiter = ' '):
    data = []
    for dictionary in dictionary_list:
        data.append(dictionary[key])
    return delimiter.join(data)

def startSimulator(topic_name):
    """
    launch the dummy odometry "simulator" in a separate thread

    Args:
        topic_name: the ros topic name to which the data is published

    """
    thread = threading.Thread(target = odometrySimulator, args=[topic_name])
    thread.daemon = True
    thread.start()
    return thread

def odometrySimulator(topic_name):
    """
    launch the dummy odometry "simulator" in a separate thread

    Args:
        topic_name: the ros topic name to which the data is published

    """
    publisher = rospy.Publisher(topic_name, Pose2D, queue_size=10)
    while True:
        pose = Pose2D()
        pose.x = np.random.uniform(0, 10)
        pose.y = np.random.uniform(0, 10)
        pose.theta = np.random.uniform(-3.14159, 3.14159)
        publisher.publish(pose)
        uncertainty = np.random.uniform(-0.005, 0.003)
        time.sleep(0.1 + uncertainty)
    return