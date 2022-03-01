from particle.filter import *
from general.utils import *
from wifi_localization.msg import rssData
from wifi_localization.srv import transformRequestService
from wifi_localization.srv import transformUpdateService

class Localizer:
    '''
    The class designed to localize the robot
    '''
    # Data folder/files
    result_folder_ = ''
    path_loss_file_ = ''
    GP_file_ = ''
    
    # Topics
    input_topic_ = "" 
    particlecloud_topic_ = ""
    pose_topic_ = ""
    
    # Frames
    map_frame_ = ""
    odometry_frame_ = ""
    base_frame_ = ""
    
    # Parameters
    x_min_ = None
    x_max_ = None
    y_min_ = None
    y_max_ = None
    n_particles_ = None
    rotational_noise_ = None
    translational_noise_ = None
    resample_threshold_ = None
    lower_limit_ = None
    
    # Subscribers/publishers
    rss_data_subscriber_ = None 
    particlecloud_publisher_ = None
    pose_publisher_ = None
    
    # Services
    request_service_=""
    update_service_=""
    
    # Bssids
    selected_bssids_= []
    
    # MCL
    mcl_ = None
    
    # Others
    last_odometry_ = None
    
    def __init__(self):
        """
        Initializer of the localizer class:
            1. load parameters
            2. load trained result (pre-selected bssids and trained sensor model parameter files) 
            3. initialize monte carlo filter
            4. initialize subscribers and publishers
        """
        # Load parameters
        self.loadParameters()
        
        # Load result
        self.loadResult()
        
        # Initialize MCL
        self.mcl_ = Filter(self.x_min_, self.x_max_, self.y_min_, self.y_max_, 
                           self.n_particles_, self.rotational_noise_, self.translational_noise_, self.resample_threshold_,
                           self.result_folder_+self.path_loss_file_, self.result_folder_+self.GP_file_+'.zip')
        
        # Initialize subscribers/publishers
        self.rss_data_subscriber_ = rospy.Subscriber(self.input_topic_, rssData, self.rssDataCallBack)
        self.particlecloud_publisher_  = rospy.Publisher(self.particlecloud_topic_, PoseArray, queue_size = 1)
        self.pose_publisher_      = rospy.Publisher(self.pose_topic_, PoseStamped, queue_size = 1)
        
        # Other variables
        self.last_odometry_ = np.array([0, 0, 0])
        
    def loadParameters(self):
        """
        Load parameters from ROS parameters server
        """
        ns = rospy.get_name()
        self.result_folder_ = rospy.get_param(ns + "/result_folder", "default_result_folder")
        
        # Topics 
        self.input_topic_ = rospy.get_param("/receiver/output_topic", "default_rss_data_topic")
        self.particlecloud_topic_ = rospy.get_param("/localizer/topics/output/particlecloud_topic", "default_particlecloud_topic")
        self.pose_topic_ = rospy.get_param("/localizer/topics/output/pose_topic", "default_pose_topic")
        
        # Frames
        self.map_frame_ = rospy.get_param("/localizer/frames/map_frame", "default_map_frame")
        self.odometry_frame_ = rospy.get_param("/localizer/frames/odometry_frame", "default_odom_frame")
        self.base_frame_ = rospy.get_param("/localizer/frames/base_frame", "default_base_frame")
        
        # MCL parameters
        self.x_min_ = rospy.get_param("/localizer/parameters/x_min", -20)
        self.x_max_ = rospy.get_param("/localizer/parameters/x_max", +20)
        self.y_min_ = rospy.get_param("/localizer/parameters/y_min", -20)
        self.y_max_ = rospy.get_param("/localizer/parameters/y_max", +20)
        self.n_particles_ = rospy.get_param("/localizer/parameters/n_particles", 3000)
        self.rotational_noise_ = rospy.get_param("/localizer/parameters/rotational_noise", 0.1)
        self.translational_noise_ = rospy.get_param("/localizer/parameters/translational_noise", 0.05)
        self.resample_threshold_ = rospy.get_param("/localizer/parameters/resample_threshold", 0.1)
        self.lower_limit_ = rospy.get_param("/localizer/parameters/lower_limit", -95)
        self.request_service_ = rospy.get_param("/service/transform_request_server", "default_update_server")
        self.update_service_ = rospy.get_param("/service/transform_update_server", "default_update_server")
        
    def retriveAction(self):
        """
        Find the odometry difference from robot base frame
        """      
        odometry = callRosService(self.request_service_, transformRequestService, [self.odometry_frame_, self.base_frame_])
        print("Current odometry data:\n", odometry)
        current_odometry = np.array([odometry.x, odometry.y, odometry.theta])
        action = current_odometry - self.last_odometry_ 
        self.last_odometry_ = current_odometry
        return action
        
    def loadResult(self):
        """
        Load trained data from the specified result path.
        """
        self.selected_bssids_ = readData(self.result_folder_ + "selections.csv")[0]
        self.path_loss_file_ = rospy.get_param("/localizer/files/path_loss_file", "pathLoss.csv")
        self.GP_file_ = rospy.get_param("/localizer/files/GP_file", "GP")
        
    def rssDataCallBack(self, message, verbose=False):
        """
        Receive rss data from input topic
            1. process rss data
            2. retrive action from odometry data
            3. use mcl to estimate pose
            4. publish pose and particles
            
        Args: 
            message: rssData type
            verbose: bool, True to publish running info
        """
        # Process rss data
        selected_rss = filterRssData(message.bssid, message.rss, self.selected_bssids_, self.lower_limit_)
        normalized_rss = normalizeRss(selected_rss, self.lower_limit_)
        
        # Retrive action
        action = self.retriveAction()
        if verbose:
            print("Message:\n", message)
            print("Selected rss:\n", selected_rss)
            print("Normalized rss:\n", normalized_rss)
            print("Action:\n", action)
        
        # Use mcl to estimate pose
        # estimate = self.mcl_.estimate(action, rss)
        # particles = self.mcl_.getParticles()
        estimate = np.array([1 , 1, 0])
        particles = np.array([1, 2, 0.1, 2, 3, 0.4, 3, 3 ,0.5, 4.5, 7.1, 1.3]).reshape(4,3)
        
        # Publish 
        self.publishPose(estimate)
        self.publishParticless(particles)
    
    def publishPose(self, estimate):
        """
        Publish pose
        
        Args:
            estimate: 1 x 3 numpy array of the form [x, y, theta]
        """
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.map_frame_
        ps.pose = self.poseFromParticle(estimate)
        self.pose_publisher_.publish(ps)
    
    def publishParticless(self, particles):
        """
        Publish particles
        
        Args:
            particles: N x 3 numpy array of the form [..., [xi, yi, thetai], [...]]
        """
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = self.map_frame_
        pa.poses = [self.poseFromParticle(particle) for particle in particles ] 
        self.particlecloud_publisher_.publish(pa)
    
    def poseFromParticle(self, particle, verbose=False):
        """
        Convert particle to pose message
        
        Args:
            particle: 1 x 3 numpy array of the form [x, y, theta]
            verbose: bool, True to publish details
            
        Return:
            pose of type geometry_msgs.msg.Pose
        """
        point = Point(particle[0], particle[1], 0)
        orientation = Quaternion(*getQuaternionFromEuler(0, 0, particle[2]))
        pose = Pose(point, orientation)
        if verbose:
            print("Particle:\n", particle)
            print("Orientation:\n", pose.orientation, '\n')
        return pose
    
    def __del__(self):
        """
        The destructor of the class
        """
        print("Localizer terminates.")

