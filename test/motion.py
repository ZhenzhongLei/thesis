from particle.motion import *
def testMotionMOdel():
    """
    Check if motion model works properly
    """   
    # Noises
    translational_noise = 0.15
    rotational_noise = 0.02 # 0.01744444444
    
    # Generate dummy particles
    n_particles = 1000
    particles = np.zeros((n_particles, 3))
    particles[:,0:2] = np.random.uniform(low=[-20, -20], high=[20, 20], size=(n_particles, 2))
    particles[:,2] = np.random.normal(loc=0.0, scale=np.math.pi/3, size=n_particles)
    
    # Visualize generated particles
    drawDistribution(particles)

    # Generate dummy action
    action = np.array([1, 1 , -0.17])
    print("Dummy action:\n", action)
    
    # See how partocles are propagated
    for i in range(10):
        model = Motion(translational_noise, rotational_noise)
        propagated_particles = model.propagate(particles, action)
        drawDistribution(propagated_particles)
    
if __name__ == "__main__":
    testMotionMOdel()