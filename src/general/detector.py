from general.utils import *

class Detector:
    # Particlecloud topics
    reference_particlecloud_topic_ = ""
    evaluate_particlecloud_topic_ = ""
    
    # Synchronizer for two particlecloud topics
    synchronizer_ = None
    
    # Parameters
    threshold_ = None
    
    def __init__(self):
        """
        Initializer, load parameters and initialize synchronizer
        """
        self.loadParameters()
        reference_particlecloud_subscriber = message_filters.Subscriber(self.reference_particlecloud_topic_, PoseArray)
        evaluate_particlecloud_subscriber = message_filters.Subscriber(self.evaluate_particlecloud_topic_, PoseArray)
        self.synchronizer_ = message_filters.ApproximateTimeSynchronizer([reference_particlecloud_subscriber, evaluate_particlecloud_subscriber], queue_size=5, slop=0.1)
        self.synchronizer_.registerCallback(self.callBack)
    
    def loadParameters(self):
        """
        Load parameters
        """
        self.threshold_ = rospy.get_param("/detector/threshold", 5)
        self.reference_particlecloud_topic_ = rospy.get_param("/localizer/topics/output/particlecloud_topic", "default_reference_particlecloud_topic")
        self.evaluate_particlecloud_topic_ = rospy.get_param("/detector/evaluate_particlecloud_topic", "default_evaluate_particlecloud_topic")

    def callBack(self, reference_particlecloud, evaluate_particlecloud):
        evaluates = self.converPoseArrayToNumpy(evaluate_particlecloud)
        references = self.converPoseArrayToNumpy(reference_particlecloud)
        metric = 0
        for evaluate in evaluates:
            probability = self.determineIfKidnapped(evaluate, references)
            metric = metric + probability

        if metric < self.threshold_:
            print("Robot is kidnapped")
    
    def converPoseArrayToNumpy(self, pose_array):
        """
        Convert the pose array to numpy array
        
        Args:
            pose_array: geometry_msgs.msg.PoseArray
        
        Return:
            array, n x 2
        """
        n_poses = len(pose_array.poses)
        array = np.zeros((n_poses, 2))
        for i in range(n_poses):
            array[i, 0] = pose_array.poses[i].position.x
            array[i, 1] = pose_array.poses[i].position.y
        return array
    
    def determineIfKidnapped(self, evaluate, references):
        """
        Determine if kidnapped situation happened or not
            
        Args: 
            evaluate: (2,) numpy array with x and y coordinates of evaluation point
            references: n x 2 numpy array, x and y coordinates of wifi localization particles 
        """
        s = references.shape[0]
        h = s ** -1/6 
        probability = np.exp(-np.sum((references - evaluate)**2, axis=1)/(2*h**2))
        probability = 1/(s*h*np.sqrt(2*np.math.pi)) * np.sum(probability)
        return probability

    def __del__(self):
        """
        The destructor of the class
        """
        print("Detector terminates.")
    
def testKidnappedDetection():
    """
    Standalone function to check pipeline of kidnap detection
    """
    reference = np.array([1, 2, 0.1, 2, 3, 0.4, 3, 3 ,0.5, 4.5, 7.1, 1.3]).reshape(6,2)
    print("Reference: \n", reference)
    s = reference.shape[0]
    print("s: \n", s)
    h = s ** -1/6
    print("h: \n", h)
    evaluates = np.array([2.5, 2, 3, 4, 5, 1]).reshape(3, 2)
    print("Evaluates: \n", evaluates)
    metric = 0
    compareClouds(reference, evaluates, "references", evaluates)
    for evaluate in evaluates:
        difference = reference - evaluate
        print("Difference: \n", difference)
        probability = np.exp(-np.sum((difference)**2, axis=1)/(2*h**2))
        print("Probability: \n", probability)
        probability = 1/(s*h*np.sqrt(2*np.math.pi)) * np.sum(probability)
        print("Probability: \n", probability)
        metric = metric + probability
    
        
if __name__ == "__main__":
    testKidnappedDetection()
