import tf
from general.utils import *
from wifi_localization.srv import transformUpdateService
from wifi_localization.srv import transformRequestService

class Service:
    """
    The service class used to provide services for this project
    """
    transform_listener_ = None
    transform_broadcaster_ = None
    transform_update_server_ = None
    transform_request_server_ = None
    transform_update_server_name_ = ""
    transform_request_server_name_ = ""
    def __init__(self):
        """
        Initializer 
            1. load parameters
            2. initialize tf broadcaster and listener
            3. initialize service servers
        """
        self.loadParameters()
        self.transform_broadcaster_ = tf.TransformBroadcaster()
        self.transform_listener_ = tf.TransformListener()
        print("Services are launched.")
        self.transform_update_server_ = rospy.Service(self.transform_update_server_name_, transformUpdateService, self.transformUpdateServiceHandler)
        self.transform_request_server_ = rospy.Service(self.transform_request_server_name_, transformRequestService, self.transformRequestServiceHandler)
    
    def loadParameters(self):
        """
        Load parameters from parameter server
        """
        self.transform_update_server_name_ = rospy.get_param("/service/transform_update_server", "default_update_server")
        self.transform_request_server_name_ = rospy.get_param("/service/transform_request_server", "default_request_server")
        
    def transformUpdateServiceHandler(self, update):
        """
        Handle transform update call
        
        Args:
            update, refer to transformRequestService for fields, (transform from child frame to parent frame)
        
        Return:
            transform between parent and child frames as specified in request message
        """
        
        self.transform_broadcaster_.sendTransform((update.x, update.y, 0),
            tf.transformations.quaternion_from_euler(0, 0, update.theta),
            rospy.Time.now(),
            update.child_frame,
            update.parent_frame)
        return True
            
    def transformRequestServiceHandler(self, request):
        """
        Handle transform request call
        
        Args:
            request, refer to transformRequestService for fields
        
        Return:
            transform between parent and child frames as specified in request message
        """
        
        try:
            (trans,rot) = self.transform_listener_.lookupTransform(request.target_frame, request.source_frame, rospy.Time(0))
            x = trans[0]
            y = trans[1]
            theta = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            theta = 0
        return [x, y, theta]
    