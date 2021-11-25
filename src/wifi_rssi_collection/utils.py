import os
import sys
import glob
import rospy
import tf2_ros
import numpy as np, copy
from rospy.msg import AnyMsg
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


