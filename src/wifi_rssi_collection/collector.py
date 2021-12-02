from wifi_rssi_collection.utils import *
from wifi_rssi_collection.iwlist import *

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
        self.pass_code_ = rospy.get_param(ns + "/pass_code", "default_code")
    
    def odometryCallBack(self, message):
        print("Data received")

        # Get current date and time
        date = datetime.datetime.now().strftime("%d-%m-%y")
        time = datetime.datetime.now().strftime("%H-%M-%S")

        # Check if relevant files exist
        self.checkFiles(date)
        x = message.x
        y = message.y

        # Retrieve AP info and rssi
        data = getRawNetworkScan('wlp4s0', password= self.pass_code_, sudo=True)
        APinfo = getAPinfo(data['output'].decode())
        
        # Save 
        self.saveTimeStamp(time)
        self.saveCoordinates(x ,y)
        self.saveAPinfo(APinfo)
        self.saveRssi(APinfo)
        return

    def checkFiles(self, date):
        directory = self.data_folder_ + date
        if not os.path.isdir(directory):
            os.mkdir(directory)
        os.chdir(directory)
        if not os.path.isfile("time.csv"):
            with open('time.csv', 'w'): pass
        if not os.path.isfile("coordinates.csv"):
            with open('coordinates.csv', 'w'): pass
        if not os.path.isfile("AP.csv"):
            with open('AP.csv', 'w'): pass
        if not os.path.isfile("rssi.csv"):
            with open('rssi.csv', 'w'): pass

    def saveTimeStamp(self, time):        
        with open('time.csv','a') as fd:
            fd.writelines(time + '\n')

    def saveCoordinates(self, x, y):
        with open('coordinates.csv','a') as fd:
            fd.writelines("{:f} {:f}".format(x, y) + '\n')

    def saveAPinfo(self, APinfo):
        APs = extractData(APinfo, "ssid", delimiter=',    ')
        with open('AP.csv','a') as fd:
            fd.write(APs + '\n')

    def saveRssi(self, APinfo):
        rssi = extractData(APinfo, "signal")
        with open('rssi.csv','a') as fd:
            fd.writelines(rssi + '\n')

    def __del__(self):
        """
        The destructor of the class
        """
        print("Collection terminates.")
