from general.utils import *
from particle.sensor import *
from particle.motion import *

class Filter:
    """
    The main body of MCL using motion model and sensor model developed so far.
    """
    # Parameters
    x_min_ = None 
    x_max_ = None
    y_min_ = None
    y_max_ = None
    n_particles_ = None
    rotational_noise_ = None
    translational_noise_ = None
    
    # Models
    motion_model_ = None
    sensor_model_ = None
    is_initialized_ = False
        
    def __init__(self, x_min, x_max, y_min, y_max, n_particles, rotational_noise, translational_noise, path_loss_file, GP_file, coordinates_file, rss_file, sensor_option):
        """
        Initialize the filter and parameters
        
        Args:
            x_min: float, x lower limit 
            x_max: float, x upper limit
            y_min: float, y lower limit
            y_max: float, y upper limit
            n_particles: positive integer, the number of particles to be sampled
            rotational_noise: the rotational noise to be added to motion model
            translational_noise: the translational noise to be added to motion model
        """
        # Parameters
        self.x_min_ = x_min
        self.x_max_ = x_max
        self.y_min_ = y_min
        self.y_max_ = y_max
        self.n_particles_ = n_particles
        self.rotational_noise_ = rotational_noise
        self.translational_noise_ = translational_noise
        
        # Models
        self.motion_model_ = Motion(self.translational_noise_, self.rotational_noise_)
        if sensor_option:
            self.sensor_model_ = Sensor()
            self.sensor_model_.loadModel(path_loss_file, GP_file)
        else:
            self.sensor_model_ = KNN()
            self.sensor_model_.loadData(coordinates_file, rss_file)
            
    def initialize(self, observation):
        """
        Initialize particles uniformly among the space and assign them equal weights
        """
        self.is_initialized_ = True
        
        # Find the probable region
        _, _, points = generatePointsOverPlane(self.x_min_, self.x_max_, self.y_min_, self.y_max_, 50, 50)
        probability_array = self.sensor_model_.predict(points, observation)
        most_probable_point = points[np.argmax(probability_array), :]
        
        # Assign particles and weights
        self.particles_ = np.zeros((self.n_particles_, 3))
        self.particles_[:, 0:2] = np.random.normal(most_probable_point, 1, (self.n_particles_, 2))
        self.weights_ = self.sensor_model_.predict(self.particles_[:, 0:2], observation)
        
    def propagateParticles(self, action):
        """
        Propagate particles 
        """
        self.particles_ = self.motion_model_.propagate(self.particles_, action) # To-do limit the range of particles

    def updateWeights(self, observation):
        """
        Update particle weights based on sensor model
        """
        self.weights_ = self.weights_*self.sensor_model_.predict(self.particles_[:, 0:2], observation)
    
    def normalizeWeights(self):
        """
        Normalize particle weights
        """
        self.weights_ = self.weights_/np.sum(self.weights_)
        
    def residualResample(self):
        """
        Performs the residual resampling algorithm used by particle filters. Based on observation that we don't need to use random numbers to select
        most of the weights. Take int(N*w^i) samples of each particle i, and then resample any remaining using a standard resampling algorithm. The purpose
        is against degeneracy problem.
        
        Reference: J. S. Liu and R. Chen. Sequential Monte Carlo methods for dynamic systems. Journal of the American Statistical Association,
        93(443):1032–1044, 1998.
        """
        indexes = np.zeros(self.n_particles_, 'i')
        
        # Get copy of items if probabilities are more significant
        copies = (np.floor(self.n_particles_*self.weights_)).astype(int)
        k = 0
        for i in range(self.n_particles_):
            for _ in range(copies[i]): 
                indexes[k] = i
                k = k + 1

        # Fill in the rest particles
        residual = self.weights_ - copies 
        residual /= sum(residual) 
        cumulative_sum = np.cumsum(residual)
        cumulative_sum[-1] = 1.
        indexes[k:self.n_particles_] = np.searchsorted(cumulative_sum, np.random.random(self.n_particles_-k))

        # Resample particles based on selected indexes
        self.particles_ = self.particles_[indexes]

    def resampleWheel(self):
        """
        Resample particles
        """
        index = np.random.randint(self.n_particles_)
        beta = 0.0
        maximum_weights = np.max(self.weights_)
        resampled_particles = np.zeros((self.n_particles_, 3))
        for i in range(self.n_particles_):
            beta = beta + np.random.rand()*2*maximum_weights
            while beta > self.weights_[index]:
                beta -= self.weights_[index]
                index = (index+1)%self.n_particles_
            resampled_particles[i] = self.particles_[index]
        self.particles_ = resampled_particles
        
    def resetWeights(self):
        """
        Reset particle weights to be equal
        """
        self.weights_ = np.ones(self.n_particles_) / float(self.n_particles_)
        
    def estimate(self, action, observation):
        """
        Estimate robot pose based on robot motion and observed data
        
        Args:
            action: action: (3,) or (3, 1)/(1, 3) numpy array, representing movement from k - 1 moment to k moment
            observation: 1 x n_ap or (n_ap, ) numpy array, normalized rss reading 
        Return:
            (3,) numpy array, the particle with highest weight
        """
        if self.is_initialized_:
            self.propagateParticles(action)
            self.updateWeights(observation)
            self.normalizeWeights()
            index = np.argmax(self.weights_)
            self.residualResample()
            self.resetWeights()
        else:
            self.initialize(observation)
            self.normalizeWeights()
            index = np.argmax(self.weights_)
        return self.particles_[index]
    
    def getParticles(self):
        """
        Return all particles
        
        Return:
            (n_particls, 3) numpy array, particles
        """
        return self.particles_

    def __del__(self):
        """
        The destructor of the class
        """
        return None
