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
    X_ = None # Position data matrix: expected to be n x 2
    Z_ = None # RSS reading matrix: expected to be n x m
    n_ap_ = None # Literally: the number of access points, positive integer

    # Path loss parameters
    path_loss_params_ = None # The parameters used to compute path loss model, expected to be 4 x n_ap_
    epsilon_ = 1e-3 # The term used to prevent distance evaluation between reference positions and access point positions from zero 
    penalty_factor_ = 9 # The term used to penalize "bad" path loss predictions

    # GP
    GP_ = None # Gaussian process model
    
    def _init__(self):
        """
        Initializer
        """
        return None
        
    def setData(self, X, Z, verbose=False):
        """
        Set the data and parameters
        
        Args: 
            X: n x 2 array, position data
            Z: n x m array, normalized rss reading
            verbose: Boolean, True to display execution details
        """
        assert X.shape[1] == 2
        assert X.shape[0] == Z.shape[0]
        self.X_ = X
        self.Z_ = Z
        self.n_ap_ = Z.shape[1]
        self.path_loss_params_ = np.zeros((4, self.n_ap_))
        if verbose:
            print("The position data has dimension of:\n", self.X_.shape)
            print("The RSSI reading has dimension of:\n", self.Z_.shape)
            print("The path loss parameters have been initialized to:\n", self.path_loss_params_)

    def setParameters(self, epsilon, penalty_factor, verbose=False):
        """
        Set the parameters
        
        Args: 
            epsilon: float, minimum value to prevent evaluation of distance between reference points and access points to be zero
            penalty_factor: float, greater than zero, used to penalize zero rss value from path loss evaluation
            verbose: Boolean, True to display execution details
        """
        assert penalty_factor > 0
        self.epsilon_ = epsilon
        self.penalty_factor_ = penalty_factor
        if verbose:
            print("Epsilon has been set to:\n", epsilon)
            print("Penalty has beeb set to:\n", penalty_factor)
    
    def optimize(self):
        """
        Calculate the path loss model and gaussian process parameters
        """
        self.initializePathLossParameters()
        self.calculatePathlossModel()
        self.calculateGP()

    def initializePathLossParameters(self, verbose=False):
        """
        Initialize path loss parameters, the potential AP position will be initialized to where the corresponding reading are 
        the biggest.
        
        Args:
            verbose: Boolean, True to display execution details
        """
        self.path_loss_params_[0, :] = 0.9
        self.path_loss_params_[1, :] = 0.5
        indexes = np.argmax(self.Z_, axis=0)
        for i in range(self.n_ap_):
            self.path_loss_params_[2:4, i] = self.X_[indexes[i], :]
        
        if verbose:
            print("The maximum indexes:\n", indexes)
            print("Initialized path loss parameters:\n", self.path_loss_params_)

    def calculatePathlossValue(self, x, parameters, epsilon, verbose=False):
        """
        Calculate the path loss of RSSi at position X from a given access point specified by parameters.
        pl =p0 - k*log(|x - x_ap|) where p0 is the signal strength at 1m, k is the decaying factor, x is location, x_ap is the ap position.
        
        Args:
            x: n x 2 numpy array, the position data
            parameters: 4 x 1 or 4 x m numpy array (in this case, x has to be 1 x 2), with each column in the form of [p0, k, x, y]
            epsilon: float, to prevent distance to be 0
            verbose: Boolean, to publish calculation details
        
        Return:
            float, or m x 1 numpy array, path loss prediction for singal access point or all access point at given location
        """
        x = x.reshape(-1, 2)
        if np.size(parameters) > 4:
            assert np.size(x) == 2
            
        p0 = parameters[0]
        k = parameters[1]
        x_ap = parameters[2:4].T
        d = np.sum((x - x_ap)**2, axis=1)**0.5 + epsilon
        pl = p0 - k*np.log10(d)
        pl = np.clip(pl, 0, 1).astype(float)
        
        # Debug only
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
        Calculate the path loss for a certain access point, constant terms have been removed.
        
        Args:
            parameters: 4 x 1 numpy array, the form [p0, k, x, y]
            args:
                [0] -> X, n x 2 numpy array, position data 
                [1] -> Z, n x 1 numpy array, RSS reading from the same access point collected at different positions.   
                [2] -> epsilon, float, used to prevent distance evaluation from being zero
                [3] -> k, penalty factor, positive number, used to penalize zero reading as those number indicate inability of network interface
                [4] -> evaluate_function, to calculate path loss
                [5] -> verbose, Boolean, True to display calculation details
        
        Return:
            float, cost
        """
        assert np.size(parameters) == 4
        
        # Unfold arguments
        X = args[0]
        Z = args[1]
        epsilon = args[2]
        k = args[3]
        evaluate_function = args[4]
        verbose = args[5]
        
        # Compute the path loss
        pl = evaluate_function(X, parameters, epsilon)
        
        # Compute the weights
        sign = Z > 0
        weights = sign + (1 - sign)*1/(1+ k)

        # Compute residual
        residual = Z - pl
        
        # Display calculation details if enabled
        if verbose: 
            print("Path loss estimation:\n", pl)
            print("Weights:\n", weights)
            print("Residual:\n", residual)
            
        return np.diag(weights).dot(residual).dot(residual)

    def calculatePathlossModel(self):
        """
        Calculate path loss parameters for each access point.
        """
        # Define the bound
        bounds = [(0.85, 0.95), (0, np.inf), (-np.inf, np.inf), (-np.inf, np.inf)]
        
        # Compute parameters for each access point
        for i in range(self.n_ap_):
            print("Compute path loss parameters for ", i+1, "th access point.")
            # Define arguments
            arg_list = (self.X_, self.Z_[:, i], self.epsilon_, self.penalty_factor_, self.calculatePathlossValue, False)
            # Optimize
            result = scipy.optimize.minimize(self.func, x0=self.path_loss_params_[:, i], bounds = bounds, args=arg_list)
            # Refill optimized parameters
            self.path_loss_params_[:, i] = result.x
        
        # ... 
        print("Optimized path loss parameters:\n", self.path_loss_params_)

    def calculateGP(self, verbose=False):
        """
        Calculate gaussian process, refer to https://gpy.readthedocs.io/en/deploy/GPy.models.html for more information about GPy
        Args:
            verbose, Boolean, True to display calculation details
        """
        Z_predict = np.zeros(self.Z_.shape)
        
        # Calculate path loss prediction for each access point
        for i in range(self.n_ap_):
            Z_predict[:,i] = self.calculatePathlossValue(self.X_, self.path_loss_params_[:, i], self.epsilon_)
            
        self.GP_ = GPy.models.GPRegression(self.X_, (self.Z_ - Z_predict))
        
        if verbose:
            print("Difference between Z predict and Z:\n", np.sum((self.Z_ - Z_predict)**2))
            print("Optimized GP:", self.GP_)

    def predict(self, x, observation=None):
        """
        Calculate gaussian process, refer to https://gpy.readthedocs.io/en/deploy/GPy.models.html for more information about GPy
        Args:
            x: 1 x 2 or (2,) numpy array, position to do prediction
            observation: 1 x n_ap or (n_ap, ) numpy array, normalized rss reading 
            
        return:
            mean, variance, probability
        """
        assert np.size(x) == 2
        x = x.reshape(1,2)
        
        # Calculate mean from path loss
        mean = self.calculatePathlossValue(x, self.path_loss_params_, self.epsilon_)
        
        # Calculate mean and variance from GP
        u, variance = self.GP_.predict(x)
        
        # Gather all information
        mean = mean + u
        sign = mean > 0.05 # Zero terms will be applied with small variance as they indicate inability of sensors
        variance = sign*variance + (1 - sign)*0.01 
        
        # Reshape
        variance = variance.reshape(self.n_ap_)
        mean = mean.reshape(self.n_ap_)
        observation = observation.reshape(self.n_ap_)
        
        # Compute probability
        distribution = scipy.stats.multivariate_normal(mean, np.diag(variance))
        probability = distribution.pdf(observation)
        return (np.clip(mean, 0, 1), variance, probability*np.math.sqrt((2*np.math.pi)**self.n_ap_))
    
    def saveModel(self, path_loss_file, gp_file):
        """
        Save training results to specified files
        Args:
            path_loss_file: string, csv file to save path loss parameters
            gp_file: string, file name with no extension, to which the traned GP model will be saved
        """
        # Save path loss parameters
        pass_loss_parameters = self.path_loss_params_.tolist()
        for i in range(4):
            if i == 0:
                option = 'w'
            else:
                option = 'a'
            line = concatenateString([str(x) for x in pass_loss_parameters[i]])
            writeLineToFile(path_loss_file, line, option)
            
        # Save GP model
        self.GP_.save_model(gp_file)
    
    def loadModel(self, path_loss_file, gp_file):
        """
        Load training results from specified files
        Args:
            path_loss_file: string, csv file wehre path loss parameters are saved
            gp_file: string, file name with ".zip" extension, in which the traned GP model is saved
        """
        # Load path_loss_file
        parameters =  readData(path_loss_file, True)
        self.n_ap_ = len(parameters[0])
        self.path_loss_params_ = np.zeros((4, self.n_ap_))
        for i in range(4):
            self.path_loss_params_[i] = np.array(parameters[i])
            
        # Load GP
        self.GP_ = GPy.core.gp.GP.load_model(gp_file)
    
    def __del__(self):
        """
        The destructor of the class
        """
        return None