from general.utils import *
from particle.sensor import *
from particle.motion import *

class Filter:
    # Parameters
    x_min_ = None
    x_max_ = None
    y_min_ = None
    y_max_ = None
    n_particles_ = None
    rotational_noise_ = None
    translational_noise_ = None
    resample_threshold_ = None
    
    # Models
    motion_model_ = None
    sensor_model_ = None
    
    
    def __init__(self, x_min, x_max, y_min, y_max, n_particles, rotational_noise, translational_noise, resample_threshold):
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
        self.resample_threshold_ = resample_threshold
        
        # Models
        self.motion_model_ = Motion(self.translational_noise_, self.rotational_noise_)
        self.sensor_model_ = Sensor()
        
        # Particles, weights, states
        self.initialize()
    
    def initialize(self):
        """
        Initialize particles uniformly among the space and assign them equal weights
        """
        self.particles_ = np.zeros((self.n_particles_, 3))
        self.weights_ = np.ones(self.n_particles_) / float(self.n_particles_)
        self.particles_[:,0:2] = np.random.uniform(low=[self.x_min_, self.y_min_], high=[self.x_max_, self.y_max_], size=(self.n_particles_, 2))
        self.particles_[:,2] = np.random.normal(loc=0.0, scale=np.math.pi/3, size=self.n_particles_)
        
    def propagateParticles(self, action):
        """
        Propagate particles 
        """
        self.particles_ = self.motion_model_.propagate(self.particles_, action) # To-do limit the range of particles

    def updateWeights(self, observation):
        """
        Update particle weights based on sensor model
        """
        for i in range(self.n_particles_):
            self.weights_[i] = self.weights_[i]*self.sensor_model_.predict(self.particles_[i, 0:2], observation)
    
    def normalizeWeights(self):
        """
        Normalize particle weights
        """
        self.weights_ = self.weights_/np.sum(self.weights_)
        
    def resample(self):
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
    
    def resetWeights(self):
        """
        Reset particle weights
        """
        self.weights_ = np.ones(self.n_particles_) / float(self.n_particles_)
        
    def estimate(self, action, observation):
        """
        Reset particle weights
        """
        self.propagateParticles(action)
        self.updateWeights(observation)
        self.normalizeWeights()
        index = np.max(self.weights_)
        if np.sum(self.weights_**2) < self.resample_threshold_:
            self.resample()
            self.resetWeights()
        return self.particles_[index]
    
    def getParticles(self):
        """
        Return all particles
        """
        return self.particles_
    
if __name__ == "__main__":
    N = 10
    weights = np.random.random(N).astype(float)
    print("Initial weights:\n", weights)
    weights = weights/np.sum(weights)
    print("Normalized weights:\n", weights)
    copies = (np.floor(N*np.asarray(weights))).astype(int)
    print("Normalized weights:\n", copies)
    k = 0
    indexes = np.zeros(N, 'i')
    for i in range(N):
        for _ in range(copies[i]): # make n copies
            indexes[k] = i
            k += 1
    print("Retrived indexes:\n", indexes)
    residual = weights - copies     
    residual /= sum(residual) 
    print("Residual:\n", residual)
    cumulative_sum = np.cumsum(residual)
    print("Cumulative sum(not fixed):\n", cumulative_sum)
    cumulative_sum[-1] = 1. 
    print("Cumulative sum:\n", cumulative_sum)
    indexes[k:N] = np.searchsorted(cumulative_sum, np.random.random(N-k))
    print(indexes)
    arr = np.zeros((N, 3))
    for i in range(N):
        arr[i,:] = i
    print(arr)
    print(arr[indexes])



    