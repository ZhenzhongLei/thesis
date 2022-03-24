from general.utils import *

class Motion:
    """
    Motion model considers that any motion can be generated from three basic movements: 
        a rotation, a translation and a second rotation 
    The propagation of motion will be based on this assumption. In order to prevent sample impoverishment, noise should be sufficient.
    """
    translation_noise_ = None
    rotational_noise_ = None
    
    def __init__(self, translation_noise, rotational_noise):
        self.setParameters(translation_noise, rotational_noise)

    def setParameters(self, translation_noise, rotational_noise):
        """
        Set the parameters
        
        Args: 
            translation_noise: float, noise added to one translation
            turn_noise: float, noise added to two rotations
        """
        self.translation_noise_ = translation_noise
        self.rotational_noise_ = rotational_noise      
        
    def propagate(self, particles, action):
        """
        Propagate particles based on historical data 
    
        Args:
            particles: n x 3 numpy array, particles to be propagated. 
            action: (3,) numpy array, representing movement from k - 1 moment to k moment       
        
        Return:
            n x 3 array, propagated particles
        """       
        action = action.reshape(3)
        
        # ...
        n_particle = particles.shape[0]

        particles[:,:] += action
        particles[:,0] += np.random.normal(loc=0.0,scale=self.translation_noise_,size=n_particle)
        particles[:,1] += np.random.normal(loc=0.0,scale=self.translation_noise_,size=n_particle)
        particles[:,2] += np.random.normal(loc=0.0,scale=self.rotational_noise_,size=n_particle)
        return particles
    
    def __del__(self):
        """
        The destructor of the class
        """
        return None