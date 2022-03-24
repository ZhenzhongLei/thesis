from particle.filter import *

def testResample():
    """
    Test the pipeline of resampling scheme
    """
    n_particles = 10
    weights = np.random.random(n_particles).astype(float)
    print("Initial weights:\n", weights)
    weights = weights/np.sum(weights)
    print("Normalized weights:\n", weights)
    copies = (np.floor(n_particles*np.asarray(weights))).astype(int)
    print("Normalized weights:\n", copies)
    k = 0
    indexes = np.zeros(n_particles, 'i')
    for i in range(n_particles):
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
    indexes[k:n_particles] = np.searchsorted(cumulative_sum, np.random.random(N-k))
    print(indexes)
    arr = np.zeros((n_particles, 3))
    for i in range(n_particles):
        arr[i,:] = i
    print(arr)
    print(arr[indexes])
    
if __name__ == "__main__":
    # testResample()
    # print(arr[indexes], np.mean(arr[indexes], axis=0))