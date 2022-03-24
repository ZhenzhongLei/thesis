from wifi_localization.msg import rssData
from wifi_localization.srv import transformRequestService
from general.utils import *
from general.scan import *

class Receiver:
    '''
    This class is designed to receive rss data, position data and resend them in ROS message.
    '''
    # Input related items
    interface_ = ""
    pass_code_ = ""
    output_topic_ = ""
    target_frame_ = ""
    source_frame_ = ""
    request_service_name_= ""
    
    # Subscribers
    odometry_subscriber_ = None 
    rss_data_publisher = None
    tf_listener_ = None
    
    # Parameters
    wait_time_ = None
    
    def __init__(self):
        """
        Initializer of the class:
            1. load parameters from ROS 
            2. initialize subscriber(s)/publisher(s)
            3. visualize the collected data as specified (will force the program to end)
            4. launch collecting procedure in a separate thread
        """
        # Load parameters
        self.loadParameters()

        # Initialize subscriber(s)/publisher(s)
        self.rss_data_publisher = rospy.Publisher(self.output_topic_, rssData, queue_size=10)
        
        # Launch collecting procedure in a separate thread
        thread = threading.Thread(target = self.resendData, args=[])
        thread.daemon = True
        thread.start()

    def loadParameters(self):
        """
        Load parameters from ROS
        """
        self.interface_ = rospy.get_param("/receiver/interface", "default_interface")
        self.pass_code_ = rospy.get_param("/receiver/pass_code", "default_pass_code")
        self.output_topic_ = rospy.get_param("/receiver/output_topic", "default_output_topic")
        self.target_frame_ = rospy.get_param("/receiver/target_frame", "default_target_frame")
        self.source_frame_ = rospy.get_param("/receiver/source_frame", "default_source_frame")
        self.wait_time_ = rospy.get_param("/receiver/wait_time", 0.5)
        self.request_service_name_ = rospy.get_param("/service/transform_request_server", "default_request_server")
        
    def resendData(self):
        """
        Collet data, when scanning finishes, the latest available odometry data will be used
        """
        while True:            
            time.sleep(self.wait_time_)
            print("\n\n\n----------------------------------------------------")
            # Start timer 
            start = time.time()
            
            # Get the scanning data
            try: 
                data = getScanWpa(self.interface_, password=self.pass_code_)
                APinfo = processWpaReturn(data['output'].decode())
            except IOError:
                continue
            
            # If only single AP data is available, dump it
            if len(APinfo) < 2:
                continue
            
            # Get the current position data
            position = self.retrievePose()
            
            # Package the data and publish
            message = self.packageData(position, APinfo)
            self.rss_data_publisher.publish(message)
            
            # Check how much time is consumed
            end = time.time()
            print(end - start) 
        return

    def packageData(self, position, APinfo):
        """
        Package positon data and scanning result into rssData message
        
        Args:
            position: tuple of the form (x, y, theta)
            APinfo: list of dictionary objects with fields "bssid", "ssid" and "signal"
        
        Return:
            rssData message
        """
        data = rssData()
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = "wifi_frame"
        data.x = position[0]
        data.y = position[1]
        data.theta = position[2]
        data.bssid = aggregateData(APinfo, "bssid")
        data.ssid = aggregateData(APinfo, "ssid")
        data.rss = aggregateData(APinfo, "signal", True)
        return data
    
    
    def retrievePose(self):
        """
        Retrieve robot pose from tf
        
        Return:
            tuple, (x, y, theta)
            or None if tf is unavailable 
        """
        value = callRosService(self.request_service_name_, transformRequestService, [self.target_frame_, self.source_frame_])
        return (value.x, value.y, value.theta)
    
    def __del__(self):
        """
        The destructor of the class
        """
        print("Receiver terminates.")
