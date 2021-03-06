from wifi_localization.msg import rssData
from general.utils import *

class Collector:
    '''
    This class is designed to collect rss ROS messages and save them into text files
    '''
    # Input related items
    input_topic_ = "" 
    data_folder_ = ""
    sub_folder_ = ""
    
    # Subscribers
    rss_data_subscriber_ = None 

    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            2. initialize subscriber(s)/publisher(s)
        """
        # Load parameters
        self.loadParameters()
        
        # Initialize subscriber(s)/publisher(s)
        self.rss_data_subscriber_ = rospy.Subscriber(self.input_topic_, rssData, self.rssDataCallBack)

    def loadParameters(self):
        """
        Load parameters from ROS
        """
        ns = rospy.get_name()
        self.input_topic_ = rospy.get_param("/receiver/output_topic", "default_rss_data_topic")
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "default_data_folder")
        self.sub_folder_ = rospy.get_param("collector/sub_folder", "default_data_folder")
        
    def rssDataCallBack(self, message):
        """
        Get the latest positioning informaton

        Args: 
            message: Pose2D type, the published 2D odometey data from localization node
        """
        self.checkFiles(self.sub_folder_)
        # Notice only items in "coordinates.csv" file are seperated by ' '.
        self.saveCoordinates(message.x, message.y, message.theta)
        self.saveBssid(concatenateString(message.bssid))
        self.saveSsid(concatenateString(message.ssid))
        self.saveRss(concatenateString([str(value) for value in message.rss]))
        
    def checkFiles(self, sub_folder):
        """
        Check if data files exist on provided subfolder. If not, create them.
        - folder: $(day)-$(month)-$(year)
            - "bssid.csv"
            - "coordinates.csv"
            - "ssid.csv"
            - "rss.csv"

        Args:
            sub_folder: string, subfolder to save data
        """
        directory = self.data_folder_ + sub_folder + "/raw"
        checkDirectory(directory)
        os.chdir(directory)
        if not os.path.isfile("bssid.csv"):
            with open('bssid.csv', 'w'): pass
        if not os.path.isfile("coordinates.csv"):
            with open('coordinates.csv', 'w'): pass
        if not os.path.isfile("ssid.csv"):
            with open('ssid.csv', 'w'): pass
        if not os.path.isfile("rss.csv"):
            with open('rss.csv', 'w'): pass

    def saveCoordinates(self, x, y, theta):
        """
        Save coordinates into "coordinates.csv"

        Args:
            x: float
            y: float
            theta: float
        """
        writeLineToFile('coordinates.csv', "{:f} {:f} {:f}".format(x, y, theta))
        
    def saveBssid(self, bssid):
        """
        Save bssid infor into "bssid.csv"

        Args:
            bssid: string, concatenated bssids
        """
        writeLineToFile('bssid.csv', bssid)

    def saveSsid(self, ssid):
        """
        Save ssid info into "ssid.csv"

        Args:
            ssid: string, concatenated ssids
        """   
        writeLineToFile('ssid.csv', ssid)

    def saveRss(self, rss):
        """
        Save rss data into "rss.csv"

        Args:
            rss: string, concatenated rss values
        """   
        writeLineToFile('rss.csv', rss)    

    def __del__(self):
        """
        The destructor of the class
        """
        print("Collection terminates.")
