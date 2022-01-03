from wifi_rssi_collection.utils import *
from wifi_rssi_collection.iwlist import *

class Collector:
    '''
    Be aware that in Python, members defined under class are actually shared by all instances.
    '''
    # General
    visaluze_mode_ = False
    visualize_data_ = False
    collect_option_ = False

    # Input related items
    input_topic_ = "" 
    pass_code_ = ""
    parent_frame_ = ""
    child_frame_ = ""

    # Subscribers
    odometry_subscriber_ = None  # ros subscriber

    # Data scanning and storage
    data_folder_ = None
    tf_listener_ = None

    # Semaphore 
    sem_ = threading.Semaphore(value=1)

    # Position data
    x_ = 0
    y_ = 0
    theta_ = 0

    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            2. initialize subscribers and publishers
            3. launch collecting procedure in a separate thread
        """
        # Load parameters
        self.loadParameters()
        
        # Initialize tf listener
        self.tf_listener_ = tf.TransformListener()

        # Initialize subscriber(s) and publisher(s)
        if self.collect_option_:
            self.odometry_subscriber_ = rospy.Subscriber(self.input_topic_, Pose2D, self.odometryCallBack)

        # Launch collecting procedure in a separate thread
        thread = threading.Thread(target = self.collect, args=[])
        thread.daemon = True
        thread.start()


    def loadParameters(self):
        """
        Load parameters from ROS
        """
        ns = rospy.get_name()
        self.visualize_data_ = rospy.get_param(ns + "/visualize_data", False)
        self.visaluze_mode_ = rospy.get_param(ns + "/visaluze_mode", False)
        self.collect_option_ = rospy.get_param(ns + "/collect_option", False)
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "~")
        self.input_topic_ = rospy.get_param(ns + "/input_topic", "default_topic")
        self.pass_code_ = rospy.get_param(ns + "/pass_code", "default_code")
        self.parent_frame_ = rospy.get_param(ns + "/parent_frame", "default_parent")
        self.child_frame_ = rospy.get_param(ns + "/child_frame", "default_child")
    
    def odometryCallBack(self, message):
        """
        Get the latest positioning informaton

        Args: 
            message: Pose2D type, the published 2D odometey data from localization node
        """
        self.sem_.acquire()
        self.x_ = message.x
        self.y_ = message.y
        self.theta_ = message.theta
        self.sem_.release()

    def collect(self):
        """
        Collet data, when scanning finishes, the latest available odometry data will be used
        """
        while True:
            # Retrieve AP info and rssi
            data = getRawNetworkScan('wlp4s0', password= self.pass_code_, sudo=True)
            APinfo = getAPinfo(data['output'].decode())
            
            # Get current date and time
            date = datetime.datetime.now().strftime("%d-%m-%y")
            time = datetime.datetime.now().strftime("%H-%M-%S")

            # Check if relevant files exist
            self.checkFiles(date)

            # Save time 
            self.saveTimeStamp(time)

            # Save coordinates
            self.sem_.acquire()
            if self.collect_option_ == False:
                self.retrievePose()
            self.saveCoordinates(self.x_ ,self.y_, self.theta_)
            self.sem_.release()

            # Save AP info and rssi
            self.saveAPinfo(APinfo)
            self.saveRssi(APinfo)
            print("Collect a new set of data!\n")
        return

    def retrievePose(self):
        try:
            (trans,rot) = self.tf_listener_.lookupTransform(self.parent_frame_, self.child_frame_, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.x_ = trans[0]
        self.y_ = trans[1]
        self.theta_ = tf.transformations.euler_from_quaternion(rot)[2]

    def checkFiles(self, date):
        """
        Check if data files exist based on provided date information. If not, create them.
        - folder: $(day)-$(month)-$(year)
            - "time.csv"
            - "coordinates.csv"
            - "AP.csv"
            - "rssi.csv"

        Args:
            date: string, having information about day, month and year
        """
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
        """
        Save time infor into "time.csv"

        Args:
            time: string
        """        
        with open('time.csv','a') as fd:
            fd.writelines(time + '\n')

    def saveCoordinates(self, x, y, theta):
        """
        Save coordinates into "coordinates.csv"

        Args:
            x: float
            y: float
            theta: float
        """   
        with open('coordinates.csv','a') as fd:
            fd.writelines("{:f} {:f} {:f}".format(x, y, theta) + '\n')

    def saveAPinfo(self, APinfo):
        """
        Save AP info into "ssid.csv"

        Args:
            APinfo: list of dictionary, returned from scanning service
        """   
        APs = extractData(APinfo, "ssid", delimiter=',    ')
        with open('AP.csv','a') as fd:
            fd.write(APs + '\n')

    def saveRssi(self, APinfo):
        """
        Save rssi data into "ssid.csv"

        Args:
            APinfo: list of dictionary, returned from scanning service
        """   
        rssi = extractData(APinfo, "signal")
        with open('rssi.csv','a') as fd:
            fd.writelines(rssi + '\n')

    def __del__(self):
        """
        The destructor of the class
        """
        print("Collection terminates.")
