from general.utils import *
from particle.sensor import *

class Process():
    '''
    The class designed to process specified data
    '''
    # Data folder 
    data_folder_ = ''
    result_folder_ = ''
    path_loss_file_ = ''
    GP_file_ = ''
    
    # Data
    coordinates_ = None
    rss_data_ = None
    bssid_data_ = None
    ssid_data_ = None
    
    #
    selected_bssid_ = None
    correspondence_ = None
    
    # FIltering parameters
    minimum_rss_value_ = -95
    minimum_threshold_ = -75
    minimum_repetition_factor_ = 0.2
    
    def __init__(self):
        """
        Load parameters from ROS parameters server
        """
        self.loadParameters()
        self.loadData()
        self.selectBssids()
        self.dumpBssids()
        Z = self.filterData()
        
        # Data for testing pipeline
        # data = readData("/home/andylei/catkin/src/thesis/data/development/mean.csv", True, ',')
        # data = np.array(data).astype(float)
        # X = data[:, 0:2]
        # Z = data[:, 2:13]
        self.trainModel(np.array(self.coordinates_)[:,0:2], Z)
        
    def loadParameters(self):
        """
        Load parameters from ROS parameters server
        """
        ns = rospy.get_name()
        # Parameters
        self.minimum_rss_value_ = rospy.get_param("/process/minimum_rss_value", -95) 
        self.minimum_threshold_ = rospy.get_param("/process/minimum_threshold", -75)
        self.minimum_repetition_factor_ = rospy.get_param("/process/minimum_repetition_factor", 0.3)
        
        # Data folder
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "default_data_folder")
        self.data_folder_ += rospy.get_param("/process/sub_folder", "/xx-xx-xx")
        
        # Results
        self.result_folder_ = rospy.get_param(ns + "/result_folder", "default_result_folder")
        self.path_loss_file_ = rospy.get_param("/process/path_loss_file", "pathLoss.csv")
        self.GP_file_ = rospy.get_param("/process/GP_file", "GP")
        
    def loadData(self):
        """
        Load data from specified path
        """
        self.coordinates_ = readData(self.data_folder_ + "coordinates.csv", True, ' ')
        self.bssid_data_ = readData(self.data_folder_ + "bssid.csv")
        self.rss_data_ = readData(self.data_folder_ + "rss.csv", True)
        self.ssid_data_ = readData(self.data_folder_ + "ssid.csv")
        
    def selectBssids(self):
        """
        Select bssids based on the number of "hits". If the number is too low, those ssids will not be considered.
        """
        # Count the occurence 
        count = {}
        correspondence = {}
        for row_index, bssid_row in enumerate(self.bssid_data_):
            for index, bssid in enumerate(bssid_row):
                if bssid in count.keys():
                    if self.rss_data_[row_index][index] > self.minimum_threshold_:
                        count[bssid] += 1
                else:
                    correspondence[bssid] = self.ssid_data_[row_index][index]
                    count[bssid] = 0
        
        # Take bssids if occurence exceeds the repetition
        self.selected_bssid_ = []
        self.correspondence_ = []
        repetition = int(len(self.rss_data_)*self.minimum_repetition_factor_)
        for key in count.keys():
            if count[key] > repetition:
                self.selected_bssid_.append(key)
                if correspondence[key] == '':
                    self.correspondence_.append(correspondence[key])
                else:
                    self.correspondence_.append(correspondence[key])
        print("Selected bssids:\n", self.selected_bssid_, '\n')
        print("Corresponding ssids:\n", self.correspondence_, '\n')
        
    def dumpBssids(self):
        """
        Drop selected bssids to the specified file. (bssid.csv under result subfolder of the project folder)
        """
        line = concatenateString(self.selected_bssid_)
        writeLineToFile(self.result_folder_ + "selections.csv", line, 'w')
        line = concatenateString(self.correspondence_)
        writeLineToFile(self.result_folder_ + "selections.csv", line, 'a')
    
    def filterData(self):
        """
        Filter the data by using selected bssids.
        """
        data = np.zeros((len(self.bssid_data_), len(self.selected_bssid_)))
        for i in range(len(self.bssid_data_)):
            row = filterRssData(self.bssid_data_[i], self.rss_data_[i], self.selected_bssid_)
            data[i, :] = np.array(row)
        return data
    
    def trainModel(self, X, Z):
        """
        Train sensor model and save results 
        
        Args:
            X: n x 2 numpy array, position data
            Z: n x m numpy array, filtered rss reading (havn't been normalized yet)
        """
        Z = normalizeRss(Z, self.minimum_rss_value_)
        sensor_model = Sensor()
        sensor_model.setData(X, Z)
        sensor_model.optimize()
        sensor_model.saveModel(self.result_folder_ +self.path_loss_file_, self.result_folder_ +self.GP_file_)
        
    def __del__(self):
        """
        The destructor of the class
        """
        print("Process terminates.")