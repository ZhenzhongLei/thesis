import os
import sys
import glob
import rssi
import rospy
import time
import threading
import numpy as np, copy
from scapy.all import sniff
from rospy.msg import AnyMsg
from subprocess import Popen, PIPE
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

def loadFingerPrintDataFromFolder(path, extension="txt"):
    """
    load finger print data from the folder

    Args:
        path: string, path to the folder, string type
        extension: string, file extension, "txt" is given by default

    Returns:
        the list of files with given extension, list of strings

    Raises:
        assertions if path is not of string type or the path couldn't be found
    """
    # Remove abnormal letters in path
    assert isString(path), "The path is not a string"
    safe_path = os.path.normpath(path)

    # Enter the folder
    assert os.path.isdir(safe_path), "The folder doesn't exist."
    file_path_list = glob.glob(safe_path + "/*" + extension)

    # Sort the list
    file_path_list = sorted(file_path_list)
    return file_path_list

def isString(data):
    """
    Check whether the data is a string

    Args:
        data: data to be checked

    Returns:
        boolean indicator
    """
    # Check if data is of string type
    if not isinstance(data, str):
        return False
    else:
        return True

def startSimulator(topic_name):
    """
    launch the 

    Args:
        interface: string, which is obtained by using command "iwconfig" from command line
        sudo: string, file extension, "txt" is given by default

    Returns:
        A dictionary with keys "output" and "error", which stores scanning result and error information respectively.
    """
    thread = threading.Thread(target = odometrySimulator, args=[topic_name])
    thread.daemon = True
    thread.start()
    return thread

def odometrySimulator(topic_name):
    publisher = rospy.Publisher(topic_name, Pose2D, queue_size=10)
    while True:
        pose = Pose2D()
        pose.x = np.random.uniform(0, 10)
        pose.y = np.random.uniform(0, 10)
        pose.theta = np.random.uniform(-3.14159, 3.14159)
        publisher.publish(pose)
        uncertainty = np.random.uniform(-0.005, 0.003)
        time.sleep(0.1 + uncertainty)
        print("Publish")
    return