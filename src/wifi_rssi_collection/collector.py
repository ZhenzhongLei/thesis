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

    # Semaphore 
    sem = threading.Semaphore(value=1)
    x = 0
    y = 0

    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            2. initialize subscribers and publishers
            3. launch collecting procedure in a separate thread
        """
        # Load parameters
        self.loadParameters()
        
        # Initialize subscriber(s) and publisher(s)
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
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "~")
        self.input_topic_ = rospy.get_param(ns + "/input_topic", "default_topic")
        self.visualize_data_ = rospy.get_param(ns + "/visualize_data", False)
        self.visaluze_mode_ = rospy.get_param(ns + "/visaluze_mode", False)
        self.pass_code_ = rospy.get_param(ns + "/pass_code", "default_code")
    
    def odometryCallBack(self, message):
        """
        Get the latest positioning informaton

        Args: 
            message: Pose2D type, the published 2D odometey data from localization node
        """
        self.sem.acquire()
        self.x = message.x
        self.y = message.y
        self.sem.release()

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
            self.sem.acquire()
            self.saveCoordinates(self.x ,self.y)
            self.sem.release()

            # Save AP info and rssi
            self.saveAPinfo(APinfo)
            self.saveRssi(APinfo)
            print("Collect a new set of data!\n")
        return

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

    def saveCoordinates(self, x, y):
        """
        Save coordinates into "coordinates.csv"

        Args:
            x: float
            y: float
        """   
        with open('coordinates.csv','a') as fd:
            fd.writelines("{:f} {:f}".format(x, y) + '\n')

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
