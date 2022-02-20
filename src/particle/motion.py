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

if __name__ == "__main__":
    translational_noise = 0.05 # ...
    rotational_noise = 0.02 # 0.01744444444
    particles = np.array([1, 2, 0.1, 2, 3, 0.4, 3, 3 ,0.5, 4.5, 7.1, 1.3]).reshape(4,3)
    print(particles)
    for particle in particles:
        print("Particle:\n", particle)
    action = np.array([0.2, 0.3 , -0.17])
    print("Dummy particles:\n", particles)
    print("Dummy action:\n", action)
    model = Motion(translational_noise, rotational_noise)
    propagated_particles = model.propagate(particles, action)
    print("Propagated particles:\n", propagated_particles)