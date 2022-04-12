import GPy
import scipy.optimize
import scipy.stats as stats
from scipy.spatial.kdtree import KDTree
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
    penalty_factor_ = 0.02 # The term used to penalize "bad" path loss predictions
    delta_ = 0.01 # Small interval for computing probability
    weight_scale_ = 2 # The scaling factor used to supress low likelihood
    
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

    def setParameters(self, epsilon, penalty_factor, delta, weight_scale, verbose=False):
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
        self.delta_ = delta
        self.weight_scale_ = weight_scale
        if verbose:
            print("Epsilon has been set to:\n", epsilon)
            print("Penalty has beeb set to:\n", penalty_factor)
            print("Delta has beeb set to:\n", delta)
            print("Weight scale has beeb set to:\n", weight_scale)
            
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
        self.path_loss_params_[1, :] = 1.5
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
        p0 = parameters[0] # Signal strength at ...
        k = parameters[1] # Decaying factor
        x_ap = parameters[2:4].T # x,y coordinats of access points
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
                [3] -> penalty, penalty factor, positive number, used to penalize zero reading as those number indicate inability of network interface
                [4] -> evaluate_function, to calculate path loss
                [5] -> verbose, Boolean, True to display calculation details
        
        Return:
            float, cost
        """
        # Unfold arguments
        X = args[0]
        Z = args[1]
        epsilon = args[2]
        penalty = args[3]
        evaluate_function = args[4]
        verbose = args[5]
        
        # Compute the path loss
        pl = evaluate_function(X, parameters, epsilon)
        
        # Compute the weights
        sign = Z <= 0.1
        deviation = sign*penalty + 0.01
        
        # Display calculation details if enabled
        if verbose: 
            print("Path loss estimation:\n", pl)
            print("Deviation:\n", deviation)
            
        log_prob = stats.norm.logpdf(pl, loc=Z, scale=deviation)
        
        return -np.sum(log_prob)

    def calculatePathlossModel(self, verbose=False):
        """
        Calculate path loss parameters for each access point.
        Args:
            verbose: Boolean, to publish execution details
        """
        # Define the bound
        bounds = [(0.8, 1), (0.5, 2.5), (-np.inf, np.inf), (-np.inf, np.inf)]
        
        # Compute parameters for each access point
        for i in range(self.n_ap_):
            if verbose:
                print("Compute path loss parameters for ", i+1, "th access point.")
            # Define arguments
            arg_list = (self.X_, self.Z_[:, i], self.epsilon_, self.penalty_factor_, self.calculatePathlossValue, False)
            # Optimize
            result = scipy.optimize.minimize(self.func, x0=self.path_loss_params_[:, i], bounds = bounds, args=arg_list)
            
            # Refill optimized parameters
            self.path_loss_params_[:, i] = result.x
        
        # ... 
        if verbose:
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
        self.GP_.optimize()
        
        if verbose:
            print("Difference between Z predict and Z:\n", np.sum(abs(self.Z_ - Z_predict)))
            print("Optimized GP:", self.GP_)

    def predict(self, x, observation, verbose=False):
        """
        Calculate gaussian process, refer to https://gpy.readthedocs.io/en/deploy/GPy.models.html for more information about GPy
            x = (x' - mean)/(variance^1/2)
            cdf(z) = 1/2*[1 + erf(z/sqrt(2))] (standard normal distribution)
        Args:
            x: n x 2  numpy array, positions to predict
            observation: 1 x n_ap or (n_ap, ) numpy array, normalized rss reading 
            verbose, Boolean, True to display calculation details
            
        return:
            mean, variance, probability
        """
        start = time.time()
        
        # Calculate mean from path loss
        mean = np.zeros((x.shape[0], self.n_ap_))
        
        # Calculate path loss prediction for each access point
        for i in range(self.n_ap_):
            mean[:,i] = self.calculatePathlossValue(x, self.path_loss_params_[:, i], self.epsilon_)
        
        # Calculate mean and variance
        u, v = self.GP_.predict(x)
        mean = np.clip(mean + u, 0, 1)
        variance = np.ones(self.n_ap_)*v
        
        # Compute probability
        standardized = (observation - mean)/variance**0.5
        probability = stats.norm.pdf(standardized)
        probability = np.product(probability, axis=1)
        probability = probability**(1/self.n_ap_)
        
        # Boost probability
        max_probability = np.max(probability)
        method = np.vectorize(lambda x: x * self.weight_scale_ if (max_probability - x) <= max_probability/2 else x / self.weight_scale_)
        probability = method(probability)
        end = time.time()
        if verbose:
            print("Prediction took ", end-start, " seconds.")
        return probability
    
    def saveModel(self, path_loss_file, gp_file):
        """
        Save training results to specified files
        Args:
            path_loss_file: string, csv file to save path loss parameters
            gp_file: string, file name with no extension, to which the traned GP model will be saved
        """
        # Save path loss parameters
        pass_loss_parameters = self.path_loss_params_.tolist()
        for i in range(len(pass_loss_parameters)):
            if i == 0:
                option = 'w'
            else:
                option = 'a'
            line = concatenateString([str(x) for x in pass_loss_parameters[i]])
            writeLineToFile(path_loss_file, line, option)
            
        # Save GP model
        self.GP_.save_model(gp_file)
        print("Model has been saves sucessfully.")
    
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
        self.path_loss_params_ = np.zeros((len(parameters), self.n_ap_))
        for i in range(len(parameters)):
            self.path_loss_params_[i] = np.array(parameters[i])
            
        # Load GP
        self.GP_ = GPy.core.gp.GP.load_model(gp_file)
    
    def __del__(self):
        """
        The destructor of the class
        """
        return None

class KNN:
    """
    KNN predictor is used to predict probability of getting a certain observation at a location
    """
    # General
    X_ = None # Position data matrix: expected to be n x 2
    Z_ = None # RSS reading matrix: expected to be n x m
    kdtree_ = None
    n_ap_ = None
    k_ = 6
    threshold_ = 1
    delta_ = 0.02
    variance_ = 0.0004
    
    def _init__(self):
        """
        Initializer
        """
        return None

    def setData(self, X, Z):
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
        self.kdtree_ = KDTree(self.X_)
        self.n_ap_ = Z.shape[1]
    
    def loadData(self, coordinates_file, rss_file):
        """
        Load data from specified data path and set internal states
        
        Args: 
            coordinates_file: string, coordinates data file corresponding to specified rss data file
            rss_file: string, processed rss data file (filtered, normalized and averaged)
        """
        X = np.array(readData(coordinates_file, True, ' '))[:, 0:2]
        Z = np.array(readData(rss_file, True))
        self.setData(X, Z)
    
    def getClosestDistance(self, x, number=1):
        """
        Return the closest distances between references and provided point(s)
        
        Args: 
            x: n x 2  numpy array, positions to query
        """
        distance_vector, _ = self.kdtree_.query(x, k=number)
        return np.array(distance_vector).reshape(-1)
    
    def predict(self, x, observation, verbose=False):
        """
        Predict the likelihood of getting observation at position x based on closest reference point
        
        Args:
            x: n x 2  numpy array, positions to predict
            observation: 1 x n_ap or (n_ap, ) numpy array, normalized rss reading 
            verbose, Boolean, True to display calculation details
            
        return:
            probability
        """
        start = time.time()
        x = x.reshape(-1, 2)
        # Calculate closet points
        distance_vector, closest_indexes = self.kdtree_.query(x, k=self.k_)

        # Reshape
        distance_vector = np.array([distance_vector]).reshape(-1)
        closest_indexes = np.array([closest_indexes]).reshape(-1)
        
        # Find references
        references = self.Z_[closest_indexes]
        
        # Compute weighted probability 
        lb = (references-observation-self.delta_)/(2*self.variance_)**0.5
        ub = (references-observation+self.delta_)/(2*self.variance_)**0.5
        probability = 1/2*(scipy.special.erf(ub)-scipy.special.erf(lb))
        probability = np.product(probability, axis=1)**(1/self.n_ap_)
        weighted_probability = np.array([self.computeWeightedProbability(probability, distance_vector, i*self.k_, (i+1)*self.k_) for i in range(x.shape[0])])
        
        end = time.time()
        if verbose:
            print("Prediction took ", end-start, " seconds.")
        return weighted_probability
    
    def computeWeightedProbability(self, probability, distance_vector, start, end):
        """
        Compute weighted probability based on distance
        """
        mask = distance_vector[start:end] <= self.threshold_
        weights = mask + (1- mask)*(0.1/(distance_vector[start:end] + 1e-2)) # 1e-2 to prevent "divide by 0"
        return np.sum(probability[start:end]*weights)
    
    def __del__(self):
        """
        The destructor of the class
        """
        return None

class Hybrid:
    """
    Hybrid predictor to estimate probability of getting a certain observation at a location
    """
    knn_ = None
    estimator_ = None
    threshold_ = 1
    
    def __init__(self):
        """
        Initializer
        """
        return None
    
    def loadModel(self, coordinates_file, rss_file, path_loss_file, gp_file):
        """
        Load data from specified data path and set internal states
        
        Args: 
            coordinates_file: string, coordinates data file corresponding to specified rss data file
            rss_file: string, processed rss data file (filtered, normalized and averaged)
            path_loss_file: string, csv file wehre path loss parameters are saved
            gp_file: string, file name with ".zip" extension, in which the traned GP model is saved
        """
        self.knn_ = KNN()
        self.knn_.loadData(coordinates_file, rss_file)
        self.estimator_ = Sensor()
        self.estimator_.loadModel(path_loss_file, gp_file)
    
    def predict(self, x, observation, verbose=False):
        """
        Predict the likelihood of getting observation at position x by using hybrid model
        
        Args:
            x: n x 2  numpy array, positions to predict
            observation: 1 x n_ap or (n_ap, ) numpy array, normalized rss reading 
            verbose, Boolean, True to display calculation details
            
        return:
            probability
        """
        x = x.reshape(-1, 2)
        distance_vector = self.knn_.getClosestDistance(x)
        probability = np.zeros(distance_vector.shape)
        knn_indexes = np.argwhere(distance_vector < self.threshold_).reshape(-1)
        estimator_indexes = np.argwhere(distance_vector >= self.threshold_).reshape(-1)
        if np.size(knn_indexes) is 0:
            probability = self.estimator_.predict(x, observation)
        elif np.size(estimator_indexes) is 0:
            probability = self.knn_.predict(x, observation)
        else:
            probability[knn_indexes] = self.knn_.predict(x[knn_indexes], observation)
            probability[estimator_indexes] = self.estimator_.predict(x[estimator_indexes], observation).reshape(-1)
        return probability