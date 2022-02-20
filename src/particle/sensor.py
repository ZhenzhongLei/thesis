import GPy
import scipy.stats
import scipy.optimize
from general.utils import *

class Sensor:
    """
    Sensor model is designed to compute the possibility of getting a certain observation at a certain position.
    In this implementation, gaussian process with path loss model as mean function is used.
    """
    # General
    X_ = None # Position data: expected to be n x 2
    Z_ = None # RSSI reading: expected to be n x m
    n_ = None # Literally: n, the number of data pairs
    m_ = None # Literally: m, also the number of access points

    # Path loss parameters
    path_loss_params_ = None # The parameters used to compute path loss model, expected to be 4 x m 
    epsilon_ = 1e-3 # The term used to prevent distance evaluation from zero 
    penalty_factor_ = 50 # The term used to penalize "bad" rssi readings
    number_of_var_ = 2 # 

    # GP
    GPs = [] # A list with m elements corresponding to m access points
    
    def _init__(self):
        """
        Initialize, basically take in data
        """
        pass
        
    def setData(self, X, Z):
        """
        Set the data and reset the parameters
        
        Args: 
            X: n x 2 array, position data
            Z: n x m array, normalized rssi reading
        """
        assert X.shape[1] == 2
        assert X.shape[0] == Z.shape[0]
        self.X_ = X
        self.Z_ = Z
        self.n_ =  X.shape[0]
        self.m_ = Z.shape[1]
        self.path_loss_params_ = np.zeros((4, self.m_))
        print("The position data has dimension of:\n", self.X_.shape)
        print("The RSSI reading has dimension of:\n", self.Z_.shape)
        print("The path loss parameters have been initialized to:\n", self.path_loss_params_)

    def setParameters(self, epsilon, penalty_factor, number_of_var):
        """
        Set the parameters
        
        Args: 
            pathloss_noise: float, the assumed noise when 
            penalty_noise: 
            number_of_var: positive integer, the number of standard deviation to remedy distribution
        """
        self.epsilon_ = epsilon
        self.penalty_factor_ = penalty_factor
        self.number_of_var_ = number_of_var
    
    def optimize(self):
        """
        Set the parameters to be optimized
        """
        self.initializePathLossParameters()
        self.calculatePathlossModel()
        self.calculateGP()

    def initializePathLossParameters(self):
        """
        Initialize path loss parameters, the potential AP position will be initialized to where the corresponding readings are 
        the biggest.
        """
        self.path_loss_params_[0, :] = 0.9
        self.path_loss_params_[1, :] = 0.5
        indexes = np.argmax(self.Z_, axis=0)
        for i in range(self.m_):
            self.path_loss_params_[2:4, i] = self.X_[indexes[i], :]
        
        print("The maximum indexes:\n", indexes)
        print("Initialized path loss parameters:\n", self.path_loss_params_)

    def calculatePathlossValue(self, x, parameters, epsilon, verbose=False):
        """
        Calculate the path loss of RSSi at position X from a given access point specified by parameters.
        pl =p0 - k*log(|x - x_ap|) where p0 is the signal strength at 1m, k is the decaying factor, x is location, x_ap is the ap position.
        
        Args:
            X: n x 2 numpy array, the position data
            parameters: 4 x 1 numpy array, the form [p0, k, x, y]
        """
        p0 = parameters[0]
        k = parameters[1]
        x_ap = parameters[2:4]
        d = np.sum((x - x_ap)**2, axis=1)**0.5 + epsilon
        pl = p0 - k*np.log10(d)
        pl = np.clip(pl, 0, 1)
        if verbose:
            print("Parameters:\n", parameters)
            print("Position vector:\n", x)
            print("AP position:\n", x_ap)
            print("Positional difference:\n", x - x_ap)
            print("Squared difference:\n", (x - x_ap)**2)
            print("Distance:\n", d)
            print("Path loss value:\n", pl)
        return pl

    @staticmethod
    def func(parameters, *args):
        """
        Calculate the path loss as described in paper, constant terms have been removed.
        
        Args:
            parameters: 4 x 1 numpy array, the form [p0, k, x, y]
            args: [0] -> j, integer, used to extract data associated with specified access point
                [1] -> X, nx2 numpy array, position data 
                [2] -> Z, n x m numpy array, RSSI readings with each column having readings from 
                        the same access point collected at different positions.   
                [3] -> epsilon, float, used to prevent distance evaluation from being zero
                [4] -> k, penalty factor, positive number, used to penalize zero readings as those number indicate inability of network interface
                [5] -> function to calculate path loss, f(x, parameters) where x is the n x 2 numpy array, postion data
        """
        j = args[0]
        X = args[1]
        Z = args[2][:, j]
        epsilon = args[3]
        k = args[4]
        evaluate_function = args[5]

        # Compute the path loss
        pl = evaluate_function(X, parameters, epsilon)
        sign = Z > 0
        weights = sign + (1 - sign)*1/(1+ np.exp(-k*pl))

        # Compute final value
        residual = Z - pl
        return np.diag(weights).dot(residual).dot(residual)

    def calculatePathlossModel(self):
        """
        Calculate path loss parameters for each access point.
        """
        for i in range(self.m_):
            arg_list = (i, self.X_, self.Z_, self.epsilon_, self.penalty_factor_, self.calculatePathlossValue)
            optimized_parameters, _, _ = scipy.optimize.fmin_l_bfgs_b(self.func, x0=self.path_loss_params_[:, i], args=arg_list, approx_grad=True)
            self.path_loss_params_[:, i] = optimized_parameters
        print("Optimized path loss parameters:\n", self.path_loss_params_)

    def calculateGP(self):
        """
        Calculate gaussian process for each access point. 
        """
        # https://gpy.readthedocs.io/en/deploy/GPy.models.html, the default kernel is rbf or squared exponential kernel
        for i in range(self.m_):
            parameter = self.path_loss_params_[:, i]
            pl = self.calculatePathlossValue(self.X_, parameter, self.Z_[:, i], self.epsilon_)
            gp = GPy.models.GPRegression(self.X_, (self.Z_[:, i] - pl).reshape(-1, 1))
            self.GPs.append(gp)
            self.GPs[i].optimize()
        print("Optimized GP:\n", self.GPs)

    def predict(self, x):
        mean = np.zeros((self.m_, 1))
        variance = np.zeros((self.m_, 1))
        for i in range(self.m_):
            pl = self.calculatePathlossValue(x, self.path_loss_params_[:, i], self.epsilon_)
            u, v = self.GPs[i].predict(x)
            mean[i] = pl + u
            variance[i] = v

        deviation =  (1./self.number_of_var_)*np.clip(self.number_of_var_*variance**0.5 + mean-  
                    np.clip(mean,0,np.inf), 1e-2, np.inf)
        return (np.clip(mean, 0, 1), deviation**2 + 0.005)
        

if __name__ == "__main__":
    # n = 10 # The number of data
    # m = 5 # The number of access points
    # k = 50 # ...
    # X = np.random.uniform(-10, 10, (n, 2))
    # Z = np.random.sample((n, m))
    # print("Input position data is:\n", X)
    # print("Input RSSI reading is:\n",Z)
    # solver = Sensor(X, Z)
    # solver.initialize_parameters()
    # pl = solver.calculatePathlossValue(solver.X_, solver.path_loss_params_[:, 0], solver.epsilon_, True)
    # sign = Z[:, 0] > 0
    # print("Rssi vector:\n", Z[:, 0])
    # print("Sign vector:\n", sign)
    # weights = np.diag(sign + (1 - sign)*1/(1+ np.exp(-k*pl)))
    # print("Weights:\n", weights)
    # residual = Z[:, 0] - pl
    # print("Residual:\n", residual, residual.shape)
    # print(weights.dot(residual).dot(residual))
    # solver.calculatePathlossModel()
    # solver.calculateGP()
    # print(solver.predict(np.array([5, 5]).reshape(1,2)))
    array = np.array([1, 2, 3, 4, 5 ,6]).reshape(2,3)
    x, y, z = array[0]
    print(x, y, z)
    print(array[0, 0])
    print(array[1, 0])