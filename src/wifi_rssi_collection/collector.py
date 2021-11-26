from wifi_rssi_collection.utils import *

class Collector:
    '''
    Be aware that in Python, members defined under class are actually shared by all instances.
    '''
    # General
    visaluze_mode_ = False
    visualize_data_ = False

    # Input related items
    input_topic_ = "" 
    pass_code_ = ""
    # Subscribers
    odometry_subscriber_ = None  # ros subscriber

    # Data scanning and storage
    data_folder_ = None

    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            3. ...
        """
        # Load parameters
        self.loadParameters()
        
        # Initialize subscriber(s) and publisher(s)
        self.odometry_subscriber_ = rospy.Subscriber(self.input_topic_, Pose2D, self.odometryCallBack)

    def loadParameters(self):
        """
        Load parameters from ROS
        """
        ns = rospy.get_name()
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "~")
        self.input_topic_ = rospy.get_param(ns + "/input_topic", "default_topic")
        self.visualize_data_ = rospy.get_param(ns + "/visualize_data", False)
        self.visaluze_mode_ = rospy.get_param(ns + "/visaluze_mode", False)
        self.pass_code_ = rospy.get_param(ns + "/pass_code", "idk")
    
    def odometryCallBack(self, message):
        print("Data received")
        data = getRawNetworkScan('wlp4s0', password= self.pass_code_, sudo=True)
        print(data)
        return

    def __del__(self):
        """
        The destructor of the class
        """
        print("Collection terminates.")
