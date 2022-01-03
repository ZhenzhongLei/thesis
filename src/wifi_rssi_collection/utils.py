import os
import sys
import glob
import rospy
import time
import tf2_ros
import tf
import datetime
import threading
import numpy as np, copy
from rospy.msg import AnyMsg
from subprocess import Popen, PIPE
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
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

def startSimulator(parameter, option):
    """
    Launch the dummy odometry "simulator" in a separate thread

    Args:
        option: boolean value, True for publishing dummy pose message and False for publishing dummy tf message
        parameter: a list, depending on option, if option is set to True, the list should contain the topic to which the pose message would be published
                if the option is set to False, the list should contain parental frame name and child frame name in order.
    """
    if option:
        thread = threading.Thread(target = odometrySimulator, args=parameter)
    else:
        thread = threading.Thread(target = transformSimulator, args=parameter)
    thread.daemon = True
    thread.start()
    return thread

def odometrySimulator(topic_name):
    """
    Launch the dummy odometry "simulator" which publishes (x, y, theta)

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

def transformSimulator(parent_frame, child_frame):
    """
    Launch the dummy transform "simulator" which only publishes random (x, y) coordinates

    Args:
        parent_frame: name of the parent frame
        child_frame: name of the child frame
    """
    br = tf.TransformBroadcaster()

    while True:
        x = np.random.uniform(0, 10)
        y = np.random.uniform(0, 10)
        theta = np.random.uniform(-np.pi, np.pi)
        br.sendTransform((x, y, 0),
            tf.transformations.quaternion_from_euler(0, 0, theta),
            rospy.Time.now(),
            child_frame,
            parent_frame)