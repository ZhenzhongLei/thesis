from general.utils import *
from wifi_localization.srv import transformUpdateService

class Simulator:
    '''
    This class is designed to provide dummy position data for testing pipeline.
    '''
    option_ = False
    dummy_robot_pose_topic_ = ""
    dummy_parent_frame_ = ""
    dummy_child_frame_ = ""
        
    def __init__(self):
        """
        Initializer:
            1. load parameters
            2. start simulation in a separate thread
        """
        self.loadParameters()
        if self.option_:
            parameter = [self.dummy_robot_pose_topic_]
        else:
            parameter = [self.dummy_parent_frame_, self.dummy_child_frame_]
        self.startSimulator(parameter)
        
    def loadParameters(self):
        """
        Load parameters from ROS
        """
        self.option_ = rospy.get_param("/simulation/option", False)
        self.dummy_robot_pose_topic_ = rospy.get_param("/simulation/dummy_robot_pose_topic", "default_robot_pose_topic")
        self.dummy_parent_frame_ = rospy.get_param("/simulation/dummy_parent_frame", "default_parent_frame")
        self.dummy_child_frame_ = rospy.get_param("/simulation/dummy_child_frame", "default_child_frame")
        
    def startSimulator(self, parameter):
        """
        Launch the dummy odometry "simulator" in a separate thread

        Args:
            parameter: a list, depending on option, if option is set to True, the list should contain the topic to which the pose message would be published
                    if the option is set to False, the list should contain parental frame name and child frame name in order.
        """
        if len(parameter) == 1:
            thread = threading.Thread(target = self.robotPoseSimulator, args=parameter)
        else:
            thread = threading.Thread(target = self.tfSimulator, args=parameter)
            
        thread.daemon = True
        thread.start()
        return thread

    def robotPoseSimulator(self, topic_name):
        """
        Launch the dummy odometry "simulator" which publishes (x, y, theta)

        Args:
            topic_name: string, the ros topic name to which the data is published
        """
        publisher = rospy.Publisher(topic_name, Pose2D, queue_size=10)
        while True:
            pose = Pose2D()
            pose.x = np.random.uniform(0, 10)
            pose.y = np.random.uniform(0, 10)
            pose.theta = np.random.uniform(-3.14159, 3.14159)
            publisher.publish(pose)
            uncertainty = np.random.uniform(-0.005, 0.003)
            time.sleep(0.1 + uncertainty)
        return

    def tfSimulator(self, parent_frame, child_frame):
        """
        Launch the dummy tf "simulator" which only publishes random (x, y) coordinates

        Args:
            parent_frame: string, the parent frame
            child_frame: string, the child frame
        """
        service_name = rospy.get_param("/service/transform_update_server", "default_update_server")
        while True:
            x = np.random.uniform(0, 10)
            y = np.random.uniform(0, 10)
            theta = np.random.uniform(-np.pi, np.pi)
            callRosService(service_name, transformUpdateService, [parent_frame, child_frame, x, y, theta])
