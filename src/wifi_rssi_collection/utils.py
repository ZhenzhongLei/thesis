import os
import sys
import glob
import rospy
import time
import datetime
import threading
import numpy as np, copy
from rospy.msg import AnyMsg
from subprocess import Popen, PIPE
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def extractData(dictionary_list, key, delimiter = ' '):
    """
    Extract wanted data from each dictionary into a string specified by given key and separated by given delimiter

    Args:
        dictionary_list: list of dictionary
        key: the key corresponding to wanted data
        delimiter: symbol, default to ' ', used to separate text 

    Returns:
        string, with wanted data separated by delimiter
    """
    data = []
    for dictionary in dictionary_list:
        data.append(dictionary[key])
    return delimiter.join(data)

def startSimulator(topic_name):
    """
    Launch the dummy odometry "simulator" in a separate thread

    Args:
        topic_name: the ros topic name to which the data is published
    """
    thread = threading.Thread(target = odometrySimulator, args=[topic_name])
    thread.daemon = True
    thread.start()
    return thread

def odometrySimulator(topic_name):
    """
    Launch the dummy odometry "simulator" in a separate thread

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