from general.utils import *
from particle.sensor import *

class Monitor:
    """
    This class is designed to provide localiztion monitoring service based on wifi rss data
    """
    # Parameters
    x_min_ = None 
    x_max_ = None
    y_min_ = None
    y_max_ = None
    
    # Models
    sensor_model_ = None
        
    def __init__(self, x_min, x_max, y_min, y_max, coordinates_file, rss_file, path_loss_file, gp_file):
        """
        Initialize the filter and parameters
        
        Args:
            x_min: float, x lower limit 
            x_max: float, x upper limit
            y_min: float, y lower limit
            y_max: float, y upper limit
            path_loss_file: positive integer, the number of particles to be sampled
            GP_file: string, file name to which 
        """
        # Parameters
        self.x_min_ = x_min
        self.x_max_ = x_max
        self.y_min_ = y_min
        self.y_max_ = y_max
        
        # Models
        self.sensor_model_ = Hybrid()
        self.sensor_model_.loadModel(coordinates_file, rss_file, path_loss_file, gp_file)
    
    def monitor(self, x, observation, krp_threshold):
        """
        Monitor particle cloud based on observation and detect if robot is kidnapped
        
        Args:
            observation: numpy array (n,) or .., observation
            particles: numpy array (m, 3), particles from lrf
            krp_threshold: float, threshold for detecting kidnapped situation
        
        Return:
            Boolean variable indicating kidnapped situation
        """
        weight = self.sensor_model_.predict(x, observation)
        print("Computed probability: ", weight)
        if weight < krp_threshold:
            return True

    def generateParticles(self, observation, n_bins=50s):
        """
        Initialize particles uniformly among the space and assign them equal weights
        
        Args:
            observation: numpy array (n,) or .., observation
            n_bins: integer, the number of bins to discretize span
        
        Return:
            initialPose, numpy array 1 x 3
        """
        # Find the probable region
        _, _, points = generatePointsOverPlane(self.x_min_, self.x_max_, self.y_min_, self.y_max_, n_bins, n_bins)
        probability_array = self.sensor_model_.predict(points, observation)
        most_probable_point = points[np.argmax(probability_array), :]
        
        # Generate particles around the most probable point
        initialPose = np.zeros((1, 3))
        initialPose[:, 0:2] = most_probable_point
        initialPose[:, 2] = 2*np.pi*np.random.rand(1) - np.pi
    
        return initialPose
    
    def __del__(self):
        """
        The destructor of the class
        """
        return None
