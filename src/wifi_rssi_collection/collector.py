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

    # Subscribers
    object_info_subscriber_ = None  # ros subscriber

    # Data storage
    data_folder_ = None

    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            2. publish static transformation between camera frame and lidar frame (for KITTI dataset mainly)
            3. ...
        """
        # Load parameters
        self.loadParameters()
        
        # Initialize subscriber(s) and publisher(s)
        self.object_info_subscriber_ = rospy.Subscriber(self.input_topic_, rospy.msg.AnyMsg, self.odometryCallBack)
    
    def loadParameters(self):
        """
        Load parameters from ROS
        """
        self.data_folder_ = rospy.get_param("/wifi_rssi_collection/data_folder", "~")
        self.input_topic_ = rospy.get_param("/wifi_rssi_collection/input_topic", "default_topic")
        self.visualize_data_ = rospy.get_param("/wifi_rssi_collection/visualize_data", False)
        self.visaluze_mode_ = rospy.get_param("/wifi_rssi_collection/visaluze_mode", False)

    def odometryCallBack(self, message):
        return

    def __del__(self):
        """
        The destructor of the class
        """
        print("Collection terminates.")
