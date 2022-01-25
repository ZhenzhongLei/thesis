from wifi_localization.utils import *

class GP:
    '''
    Be aware that in Python, members defined under class are actually shared by all instances.
    '''
    # General
    X_ = None # Position data: expected to be n x 2
    Z_ = None # RSSI reading: expected to be n x m
    n_ = None # Literally: n, the number of data pairs
    m_ = None # Literally: m, also the number of access points

    # Path loss parameters
    path_loss_params_ = None # The parameters used to compute path loss model, expected to be 4 x m 
    add_noise_var_ = 0.005 # The added noise for panelizing zero values 
    number_of_var_ = 2 # 

    # GP
    gp = None

    def __init__(self, X, Y):
        """
        Initialize, basically take in data
        
        Args: 
            X: n x 2 array, position data
            Y: n x m array, normalized rssi readings with floats ranging from 0 to 1
        """
        self.set_data(X, Y)
        
    def set_data(self, X, Y):
        """
        Set the data and reset the parameters
        
        Args: 
            X: n x 2 array, position data
            Y: n x m array, normalized rssi reading
        """
        assert X.shape[1] == 2
        assert X.shape[0] == Y.shape[0]
        self.X_ = X
        self.Y_ = Y
        self.n_ =  X.shape[0]
        self.m_ = Y.shape[1]
        self.path_loss_params_ = np.zeros(4, self.m_)
    
    def set_parameters(self, add_noise_var, number_of_var):
        """
        Set the parameters to be optimized
        
        Args: 
            add_noise_var: float, the added noise to penalize 
            number_of_var: positive integer, the number of standard deviation to remedy distribution
        """
        self.add_noise_var_ = add_noise_var
        self.number_of_var_ = number_of_var
    
    def initialize_parameters(self):
            pass
    
    def calculate_path_loss(self):
        """
        Calculate path loss parameters for each 
        """
        def func(parameters, *args):
            """
            Calculate the path loss as described in paper, constant terms have been removed.
            
            Args:
                parameters: 4 x 1 numpy array, the form [p0, k, x, y]
                args: [0] -> j, integer, used to extract data associated with specified access point
                    [1] -> X, nx2 numpy array, position data 
                    [2] -> Z, n x m numpy array, RSSI readings with each column having readings from 
                            the same access point collected at different positions.   
                    [3] -> noise, float, assumed gaussian noise, 
            """
            j = args[0]
            X = args[1]
            Z = args[2][:, j]
            noise = args[3][:, j]

            # Compute the path loss
            p0 = parameters[0]
            k = parameters[1]
            x = parameters[2:3]
            pl = p0 - k/2*np.log10(np.sum((X - x)**2, axis=1)) 
            pl = np.clip(pl, 0, p0) 

            # Compute loglikelihood based on following fomula
            '''
             - f(x) = exp(-y(x)^2/2)/(sqrt(2)*pi)
             - y = (x - loc)/scale 
             - log(f(x)) = -y(x)^2/2 - log(sqrt(2)*pi)
            '''
            logpdf = scipy.stats.norm.logpdf(Z, loc=pl, scale=noise)
            return -np.sum(logpdf)

        for i in range(self.m_):
            parameters, minimum, info = scipy.optimize.fmin_l_bfgs_b(func, x0=self.path_loss_params_[:, i], args=(i, self.X_, self.Z_), approx_grad=True)
            self.path_loss_params_[:, i] = parameters
            pass

    def calculate_GP(self):
        # https://gpy.readthedocs.io/en/deploy/GPy.models.html
        self.gp = GPy.models.GPRegression(self.ndata['X'],self.ndata['Y'])
        self.gp.optimize()
        pass

    def save_model(self):
        pass

    def load_model(self):
        pass

    def predict():
        pass