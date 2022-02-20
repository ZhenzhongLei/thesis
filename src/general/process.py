from general.utils import *

class Process():
    '''
    The class designed to process specified data
    '''
    # Data folder 
    data_folder_ = ''
    model_folder_ = ''
    
    # Data
    coordinates_ = None
    rss_data_ = None
    bssid_data_ = None
    ssid_data_ = None
    
    #
    selected_bssid_ = None
    correspondence_ = None
    
    # FIltering parameters
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
        self.filterData()
        
    def loadParameters(self):
        """
        Load parameters from ROS parameters server
        """
        ns = rospy.get_name()
        self.minimum_threshold_ = rospy.get_param("/process/minimum_threshold", -75)
        self.minimum_repetition_factor_ = rospy.get_param("/process/minimum_repetition_factor", 0.3)
        self.model_folder_ = rospy.get_param(ns + "/model_folder", "default_model_folder")
        self.data_folder_ = rospy.get_param(ns + "/data_folder", "default_data_folder")
        self.data_folder_ += rospy.get_param("/process/sub_folder", "/xx-xx-xx")
        
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
        print(self.selected_bssid_)
        print(self.correspondence_)
        
    def dumpBssids(self):
        """
        Drop selected bssids to the specified file. (bssid.csv under model subfolder of the project folder)
        """
        line = concatenateString(self.selected_bssid_)
        writeLineToFile(self.model_folder_ + "selections.csv", line, 'w')
        line = concatenateString(self.correspondence_)
        writeLineToFile(self.model_folder_ + "selections.csv", line, 'a')
    
    def filterData(self):
        data = np.zeros((len(self.bssid_data_), len(self.selected_bssid_)))
        for i in range(len(self.bssid_data_)):
            row = filterRssData(self.bssid_data_[i], self.rss_data_[i], self.selected_bssid_)
            data[i, :] = np.array(row)
        print(np.array(data))
    
    def __del__(self):
        """
        The destructor of the class
        """
        print("Process terminates.")