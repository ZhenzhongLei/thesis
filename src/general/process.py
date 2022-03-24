from general.utils import *
from particle.sensor import *

'''
    To do: spatially filter data quadtree or maybe something else
'''
class Process():
    '''
    The class designed to process specified data
    '''
    # Data folder, consisting of "/raw/" and "/result/", put under "../data"
    raw_data_folder_ = '' # Folder where data are saved
    result_data_folder = '' # Folder 
    path_loss_file_ = '' # File to save path loss parameters
    GP_file_ = '' # File to save GP parameters
    
    # Folder "../result"
    result_folder_ = '' # Folder where result should go
    
    # Filter mode
    filter_option_ = False
    filter_folder_ = ''
    
    # Data
    coordinates_ = None
    rss_data_ = None
    bssid_data_ = None
    ssid_data_ = None
    
    # Selected MACs and corresponding SSIDs
    selected_bssids_ = None
    correspondences_ = None
    
    # FIltering parameters
    process_option_ = True
    training_option_ = True
    save_model_option_ = True
    
    # Parameters
    scale_x_ = 0.25
    scale_y_ = 0.25
    minimum_rss_value_ = -95
    minimum_threshold_ = -75
    minimum_repetition_factor_ = 0.2
    
    def __init__(self):
        """
        Load parameters from ROS parameters server
        """
        self.loadParameters()
        
        # Check if "/result" under subfolder exists or not
        checkDirectory(self.result_data_folder_)
        if self.filter_option_:
            self.loadRawData()
            self.loadBssids()
            self.filterAndSaveRawData()
            return
        # ...
        if self.process_option_:
            self.loadRawData()
            self.selectBssids()
            self.dumpBssids()
            filtered_data = self.filterAndSaveRawData()
            self.averageAndSaveData(filtered_data)
        else:
            X, Z = self.loadTrainingData()
            self.trainAndSaveModel(X, Z)
        
    def loadParameters(self):
        """
        Load parameters from ROS parameters server
        """
        ns = rospy.get_name()
        # Parameters
        self.process_option_ = rospy.get_param("/process/options/process_option", True) 
        self.training_option_ = rospy.get_param("/process/options/training_option", True) 
        self.save_model_option_ = rospy.get_param("/process/options/save_model_option", True)
        
        # Parameters
        self.minimum_rss_value_ = rospy.get_param("/process/parameters/minimum_rss_value", -95) 
        self.minimum_threshold_ = rospy.get_param("/process/parameters/minimum_threshold", -75)
        self.minimum_repetition_factor_ = rospy.get_param("/process/parameters/minimum_repetition_factor", 0.3)
        self.scale_x_ = rospy.get_param("/process/parameters/scale_x", 0.3)
        self.scale_y_ = rospy.get_param("/process/parameters/scale_y", 0.3)
        
        # Data folders
        data_folder = rospy.get_param(ns + "/data_folder", "default_data_folder")
        sub_folder = rospy.get_param("/process/general/sub_folder", "/xx-xx-xx")
        self.raw_data_folder_ = data_folder + sub_folder + "raw/"
        self.result_data_folder_ = data_folder + sub_folder + "result/"
        self.result_folder_ = rospy.get_param(ns + "/result_folder", "default_result_folder")
        
        # File names
        self.path_loss_file_ = rospy.get_param("/process/general/path_loss_file", "pathLoss.csv")
        self.GP_file_ = rospy.get_param("/process/general/GP_file", "GP")
        
        # Filtering related
        self.filter_option_ = rospy.get_param("/process/filter/enable", False)
        self.filter_folder_ = data_folder + rospy.get_param("/process/filter/source", "default_filter_folder") + "result/"
        
    def loadRawData(self):
        """
        Load raw data from specified path
        """
        self.coordinates_ = readData(self.raw_data_folder_ + "coordinates.csv", True, ' ')
        self.bssid_data_ = readData(self.raw_data_folder_ + "bssid.csv")
        self.rss_data_ = readData(self.raw_data_folder_ + "rss.csv", True)
        self.ssid_data_ = readData(self.raw_data_folder_ + "ssid.csv")
    
    def loadTrainingData(self):
        """
        Load training data for sensor model
        """
        if self.training_option_:
            X = np.array(readData(self.result_data_folder_ + "averaged_coordinates.csv", True, ' '))[:, 0:2]
            Z = np.array(readData(self.result_data_folder_ + "averaged_data.csv", True))
        else:
            X = np.array(readData(self.raw_data_folder_ + "coordinates.csv", True, ' '))[:, 0:2]
            Z = np.array(readData(self.result_data_folder_ + "data.csv", True))
        return (X, Z)
    
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
        self.selected_bssids_ = []
        self.correspondences_ = []
        repetition = int(len(self.rss_data_)*self.minimum_repetition_factor_)
        for key in count.keys():
            if count[key] > repetition and int(correspondence[key][-5:-1]) > 5000:
                self.selected_bssids_.append(key)
                self.correspondences_.append(correspondence[key])
                
        print("Selected bssids:\n", "Number - ", len(self.selected_bssids_), '\n', self.selected_bssids_, '\n')
        print("Corresponding ssids:\n", self.correspondences_, '\n')
        
    def dumpBssids(self):
        """
        Drop selected bssids to the specified file. (bssid.csv under result subfolder of the project folder)
        """
        if self.save_model_option_:
            writeLineToFile(self.result_folder_ + "selections.csv", concatenateString(self.selected_bssids_), 'w')
            writeLineToFile(self.result_folder_ + "selections.csv", concatenateString(self.correspondences_), 'a')

        # Save to "result" folder under subfolder
        writeLineToFile(self.result_data_folder_ + "selections.csv", concatenateString(self.selected_bssids_), 'w')
        writeLineToFile(self.result_data_folder_ + "selections.csv", concatenateString(self.correspondences_), 'a')
    
    def loadBssids(self):
        """
        Load selected bssids to filter data
        """
        self.selected_bssids_ = readData(self.filter_folder_ + "selections.csv")[0]
        
    def filterAndSaveRawData(self):
        """
        Filter the data by using selected bssids and then save them.
            1. filter, reformat the data by using selected bssids
            2. normalize the filtered data
            3. save filtered data
        """
        # Filter data
        data = np.array([filterRssData(self.bssid_data_[i], self.rss_data_[i], self.selected_bssids_) for i in range(len(self.bssid_data_))])

        # Normalize data
        data = normalizeRss(data, self.minimum_rss_value_)
        
        # Save data
        saveArrayToFile(self.result_data_folder_ + "data.csv", data)
        return data
    
    def averageAndSaveData(self,  normalized_data):
        # Take in coordinates data
        coordinates = np.array(self.coordinates_)[:, 0:2]
        
        # Partition the spanning space into small bins
        min_x, min_y = np.min(coordinates, axis=0)
        max_x, max_y = np.max(coordinates, axis=0)
        x_number = np.floor((max_x - min_x)/self.scale_x_).astype(np.int) + 1 # Considering the 0 case
        y_number = np.floor((max_y - min_y)/self.scale_y_).astype(np.int) + 1 
        grid = np.empty((x_number, y_number), dtype=object)
        
        # Compute the location of data in the grid
        positions = coordinates - np.array([min_x, min_y])
        locations = np.trunc(positions/np.array([self.scale_x_, self.scale_y_])).astype(np.int)
        
        # Count which point locates in which bin
        for i in range(locations.shape[0]):
            x, y = locations[i]
            if grid[x, y] is None:
                grid[x, y] = [i]
            else:
                grid[x, y].append(i)
        
        # Interate through data to compute average
        is_start = False
        for x in range(x_number):
            for y in range(y_number):
                if grid[x, y] is None:
                    continue
                else:
                    mean_coordinate = np.mean(coordinates[grid[x, y]], axis=0).reshape(1, -1)
                    mean_data = np.mean(normalized_data[grid[x, y]], axis=0).reshape(1, -1)
                    if is_start:
                        averaged_coordinates = np.concatenate((averaged_coordinates, mean_coordinate), axis = 0)
                        averaged_data = np.concatenate((averaged_data, mean_data), axis = 0)
                    else:
                        is_start = True
                        averaged_coordinates = mean_coordinate
                        averaged_data = mean_data
                    
        # Save
        if self.save_model_option_:
            saveArrayToFile(self.result_folder_ + "averaged_coordinates.csv", averaged_coordinates, ' ')
            saveArrayToFile(self.result_folder_ + "averaged_data.csv", averaged_data)
        saveArrayToFile(self.result_data_folder_ + "averaged_coordinates.csv", averaged_coordinates, ' ')
        saveArrayToFile(self.result_data_folder_ + "averaged_data.csv", averaged_data)
            
    def trainAndSaveModel(self, X, Z):
        """
        Train sensor model and save results 
        
        Args:
            X: n x 2 numpy array, position data
            Z: n x m numpy array, noramlized and filtered rss reading 
        """
        sensor_model = Sensor()
        sensor_model.setData(X, Z)
        sensor_model.optimize()
        sensor_model.saveModel(self.result_data_folder_ +self.path_loss_file_, self.result_data_folder_ +self.GP_file_)
        if self.save_model_option_:
            sensor_model.saveModel(self.result_folder_ +self.path_loss_file_, self.result_folder_ +self.GP_file_)
        
    def __del__(self):
        """
        The destructor of the class
        """
        print("Process terminates.")