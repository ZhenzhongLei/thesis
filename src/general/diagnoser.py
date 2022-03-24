from monitor.monitor import *
from general.utils import *
from wifi_localization.msg import rssData
from wifi_localization.srv import transformRequestService

class Diagnoser:
    '''
    The class designed to localize the robot
    '''
    # Data folder/files
    result_folder_ = ''
    coordinates_file_ = ''
    rss_file_ = ''
    path_loss_file_ = ''
    GP_file_ = ''
    
    # Input topics
    input_rss_data_topic_ = "" 
    
    # Frames
    map_frame_ = ""
    base_frame_ = ""
    
    # Services
    request_service_ = ""
    
    # Parameters
    count_ = 0
    x_min_ = None
    x_max_ = None
    y_min_ = None
    y_max_ = None
    krp_threshold_ = None
    krp_count_ = None
    minimum_rss_value_ = None
    
    # Subscribers
    input_rss_data_subscriber_ = None
    
    # Publishers
    relocalize_publisher_ = None
    
    # Bssids
    selected_bssids_= []
    
    # Monitor
    monitor_ = None
    
    def __init__(self):
        """
        Initializer of the diagnoser class:
            1. load parameters
            2. load trained result (pre-selected bssids and trained sensor model parameter files) 
            3. initialize monitor
            4. initialize synchronizer and publishers
        """
        # Load parameters
        self.loadParameters()
        
        # Load result
        self.loadResult()
        
        # Initialize monitor
        self.monitor_ = Monitor(self.x_min_, self.x_max_, self.y_min_, self.y_max_,
                           self.coordinates_file_, self.rss_file_, self.path_loss_file_, self.GP_file_)
        
        # Initialize synchronizer
        self.input_rss_data_subscriber_ = rospy.Subscriber(self.input_rss_data_topic_, rssData, self.callBack)
    
    def loadParameters(self):
        """
        Load parameters from ROS parameters server
        """
        ns = rospy.get_name()
        self.result_folder_ = rospy.get_param(ns + "/result_folder", "default_result_folder")
        
        # Input topics 
        self.input_rss_data_topic_ = rospy.get_param("/receiver/output_topic", "default_rss_data_topic")
        
        # Frames
        self.map_frame_ = rospy.get_param("/diagnoser/frames/map_frame", "default_map_frame")
        self.base_frame_ = rospy.get_param("/diagnoser/frames/base_frame", "default_base_frame")
        
        # Services
        self.request_service_ = rospy.get_param("/service/transform_request_server", "default_request_server")
        
        # MCL parameters
        self.x_min_ = rospy.get_param("/diagnoser/parameters/x_min", -20)
        self.x_max_ = rospy.get_param("/diagnoser/parameters/x_max", +20)
        self.y_min_ = rospy.get_param("/diagnoser/parameters/y_min", -20)
        self.y_max_ = rospy.get_param("/diagnoser/parameters/y_max", +20)
        self.minimum_rss_value_ = rospy.get_param("/diagnoser/parameters/minimum_rss_value", -100)
        self.krp_threshold_ = rospy.get_param("/diagnoser/parameters/krp_threshold", 0.01)
        self.krp_count_ = rospy.get_param("/diagnoser/parameters/krp_count", 0.01)
        
    def loadResult(self):
        """
        Load trained data from the specified result path.
        """
        self.selected_bssids_ = readData(self.result_folder_ + "selections.csv")[0]
        self.coordinates_file_ = self.result_folder_ + "averaged_coordinates.csv"
        self.rss_file_ = self.result_folder_ + "averaged_data.csv"
        self.path_loss_file_ = self.result_folder_ + rospy.get_param("/diagnoser/files/path_loss_file", "pathLoss.csv")
        self.GP_file_ = self.result_folder_ + rospy.get_param("/diagnoser/files/GP_file", "GP")
        
    def callBack(self, rss_data_message, verbose=False):
        """
        Receive rss data from input topic
            1. process rss data
            2. fetch position data
            3.estimate probability
            
        Args: 
            message: rssData, rss data from ...
            verbose: bool, True to publish running info
        """
        # Process rss data
        selected_rss = filterRssData(rss_data_message.bssid, rss_data_message.rss, self.selected_bssids_, self.minimum_rss_value_)
        normalized_rss = normalizeRss(selected_rss, self.minimum_rss_value_)
        position = callRosService(self.request_service_, transformRequestService, [self.map_frame_, self.base_frame_])
        flag = self.monitor_.monitor(np.array([position.x, position.y]), normalized_rss, self.krp_threshold_)
        if flag:
            self.count_ = self.count_ + 1
        else:
            self.count_ =  0
        
        if self.count_ >= self.krp_count_:
            print("Kidnap detected~")
        
        # Publish info
        if verbose:
            print("Selected rss:\n", selected_rss)
            print("Normalized rss:\n", normalized_rss)
    
    def __del__(self):
        """
        The destructor of the class
        """
        print("Localizer terminates.")

