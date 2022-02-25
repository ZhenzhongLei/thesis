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
            particles: n x 3 array, to be propagated. 
            states: 2 x 3 array, representing poses at k - 1 moment and k moment       
        
        Return:
            n x3 array, propagated particles
        """       
        # Calculate the difference
        cosines = np.cos(particles[:,2])
        sines = np.sin(particles[:,2])
        
        # ...
        n_particle = particles.shape[0]
        local_deltas = np.zeros((n_particle, 3))
        local_deltas[:,0] = cosines*action[0] - sines*action[1]
        local_deltas[:,1] = sines*action[0] + cosines*action[1]
        local_deltas[:,2] = action[2]

        particles[:,:] += local_deltas
        particles[:,0] += np.random.normal(loc=0.0,scale=self.translation_noise_,size=n_particle)
        particles[:,1] += np.random.normal(loc=0.0,scale=self.translation_noise_,size=n_particle)
        particles[:,2] += np.random.normal(loc=0.0,scale=self.rotational_noise_,size=n_particle)
        return particles

def testMotionMOdel():
    """
    Check if motion model works properly
    """   
    # Noises
    translational_noise = 0.15
    rotational_noise = 0.02 # 0.01744444444
    
    # Dummy particles
    n_particles = 1000
    particles = np.zeros((n_particles, 3))
    particles[:,0:2] = np.random.uniform(low=[-20, -20], high=[20, 20], size=(n_particles, 2))
    particles[:,2] = np.random.normal(loc=0.0, scale=np.math.pi/3, size=n_particles)
    drawDistribution(particles)

    # Dummy action
    action = np.array([1, 1 , -0.17])
    print("Dummy action:\n", action)
    
    # Dummy model
    for i in range(10):
        model = Motion(translational_noise, rotational_noise)
        propagated_particles = model.propagate(particles, action)
        drawDistribution(propagated_particles)
    
if __name__ == "__main__":
    testMotionMOdel()