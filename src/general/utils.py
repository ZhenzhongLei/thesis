import os
import sys
import glob
from xml.dom import NotSupportedErr
import rospy
import time
import csv
import datetime
import threading
import numpy as np, copy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


def aggregateData(list, key, numeric=False):
    """
    Extract wanted data from each dictionary into a string specified by given key and separated by given delimiter

    Args:
        list: list of dictionary
        key: the key corresponding to wanted data
        delimiter: symbol, default to ' ', used to separate text 

    Returns:
        list 
    """
    data = []
    for item in list:
        unit = item[key].encode('ascii')
        if numeric:
            unit = int(unit)
        data.append(unit)
    return data

def concatenateString(data, delimiter='\t'):
    """
    Concatenate string list by using proviced delimeter

    Args:
        data: list of strings
        delimiter: symbol, default to ' ', used to separate text 

    Returns:
        Concatenated strings separated by delimeter
    """
    return delimiter.join(data)
    
def writeLineToFile(file, data, option='a'):
    """
    Write a line of data to the specified file

    Args:
        file: string, name of the specified file
        data: string, the data to be write to file
        option: character, 'a' to append, 'w' to overwrite
    """
    with open(file, option) as f:
        f.writelines(data + '\n')

def readData(file, numeric=False, separator='\t'):
    """
    Read the numerical data from a csv file

    Args:
        file: string, name of the csv file
        numeric: Boolean, if set to True, the reader will convert strings to floats
        separator: string, used to separate 
    
    Return:
        list with each entity representing one row of data
        
    Raise:
        NotSupportedErr if the file is not in csv form.
    """
    # Check the file format
    if file.split('.')[1] != 'csv':
        raise NotSupportedErr
    
    # Open the file
    f = open(file)
    
    # Extract content based on type of data
    if numeric:
        quote = csv.QUOTE_NONNUMERIC
    else:
        quote = csv.QUOTE_NONE
    reader = csv.reader(f, quoting=quote, delimiter=separator)
    
    # Format and return
    data = []
    for row in reader:
        data.append(row[:])
    return data

def filterRssData(bssid_data, rss_data, keys, minimum_rss_value=-95):
    """
    Filter the rss data by using pre-selected bssids.
    
    Args:
        bssid_data: list of strings with each representing bssid of different access point
        rss_data: list of integers with each representing signal strength from different access point
        keys: list of strings, pre-selected bssids
        minimun_rss_value: negative integer, the minimum signal strength, it is used as defualt value if some of pre-selected bssids could not
        be found.
    
    Return:
        list of integers, ordered as pre-selected bssids
    """
    # Initialize the filtered data by minimum rss values
    data = {key:minimum_rss_value for key in keys}
    
    # Fill in the data if bssid falls in pre-selected bssids.
    for index, bssid in enumerate(bssid_data):
        if bssid in keys:
            data[bssid] = rss_data[index]
    return list(data.values())

def normalizeRss(rss, minimum_rss_value):
    """
    Filter the rss data by using pre-selected bssids.
    
    Args:
        rss: list of integers, signal strength from various access points
        minimun_rss_value: negative integer, the minimum signal value
    Return:
        float, between 0 to 1 with 0 representing minimum value and 1 representing maximum signal value
    """
    return (np.asarray(rss) - minimum_rss_value)/float(np.abs(minimum_rss_value))

def getQuaternionFromEuler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Args:
        roll: The roll (rotation around x-axis) angle in radians.
        pitch: The pitch (rotation around y-axis) angle in radians.
        yaw: The yaw (rotation around z-axis) angle in radians.
    
    Return:
        return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

def callRosService(service_name, ros_service_class, parameters, verbose=False):
    """
    Call ROS service
    
    Args:
        service_name: string, name of service 
        ros_service_class: ros service class
        parameters: list of parameters to be passed to service handler (refer to service.py for details)
    
    Return:
        the return value from service handler (refer to .srv for details)
    """
    start = time.time()
    # Wait for service to be available (will block the execution)
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, ros_service_class)
    
    # Call the service to process data
    try:
        response = service(*parameters)
    except rospy.ServiceException as exc:
        response = None
        print("Service did not process request: " + str(exc))
    end = time.time()
    
    # Print the time comsumed during call of service
    if verbose:
        print("Calling service ", service_name, " consumes:\n", end - start, " seconds") 
    return response
    
def drawDistribution(poses):
    """
    Plot the coordinates data to visualize its distribution

    Args:
        poses: list, the poses collected
    """
    poses = np.array(poses)
    xpoints = poses[:, 0]
    ypoints = poses[:, 1]
    plt.plot(xpoints, ypoints, 'o')
    plt.show()
    