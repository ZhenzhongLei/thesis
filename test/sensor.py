from random import random
from particle.sensor import *

def testPathLossCalculation():
    """
    Test fucntion for testing path loss calculation of the sensor model
    """
    print("Test path loss calculation:")
    sensor_model = Sensor()
    sensor_model.calculatePathlossValue(np.array([1, 2, 4, 4]).reshape(2,2), np.array([1, 0.5, 2, 3]), 1e-3, True)
    print('\n')
    
def testObjectiveFunction():
    """
    Test fucntion for testing objective calculation of the sensor model
    """
    print("Test objective function:")
    X = np.random.random((10, 2))
    Z = np.random.random(10)
    parameters = np.array([1, 0.5, 2, 3])   
    sensor_model = Sensor()
    value = sensor_model.func(parameters, X, Z, sensor_model.epsilon_, sensor_model.penalty_factor_, sensor_model.calculatePathlossValue, True)
    print("Cost value:\n", value, '\n')

def testSensorModel(X, Z, path_loss_file, gp_file):
    """
    Test fucntion for testing sensor model training
    Args:
        X: n x 2 numpy array, position data
        Z: n x m numpy array, normalized rss data
        path_loss_file: string, file to save trained path loss model
        gp_file: string, file to save trained GP model
    """
    print("Test sensor model:")
    drawDistribution(X)
    
    # Compute path loss parameters
    sensor_model = Sensor()  
    sensor_model.setData(X, Z)
    sensor_model.initializePathLossParameters()
    sensor_model.calculatePathlossModel()
    compareClouds(X, sensor_model.path_loss_params_[2:4,:].T, "reference postions", "inferred AP positions")
    
    # Calculate GP
    sensor_model.calculateGP()
    
    # Predict and save model
    index = 50
    probability = sensor_model.predict(X[index - 1:index + 1, :].reshape(-1, 2), Z[index], True)
    print("Probability:\n", probability)
    print('\n')
    sensor_model.saveModel(path_loss_file, gp_file)
    
def testModelSaveAndLoad(X, Z, path_loss_file, gp_file):
    """
    Test fucntion for testing sensor model training
    Args:
        X: n x 2 numpy array, position data
        Z: n x m numpy array, normalized rss data
        path_loss_file: string, file to load trained path loss model
        gp_file: string, file to load trained GP model
    """
    print('Test model saving and loading')    
    # Load model
    sensor_model = Sensor() 
    sensor_model.loadModel(path_loss_file, gp_file)
    
    # Predict and compare
    index = 50
    probability = sensor_model.predict(X[index - 1:index + 1, :].reshape(-1, 2), Z[index], True)
    print("Probability:\n", probability)
     
def testModelPerformance(X, Z, path_loss_file, gp_file):
    """
    Test fucntion for testing sensor model training
    Args:
        X: n x 2 numpy array, position data
        Z: n x m numpy array, normalized rss data
        path_loss_file: string, file to load trained path loss model
        gp_file: string, file to load trained GP model
    """
    print('Test model performance')
    # Convert data
    N = X.shape[0]
    
    # Load model
    sensor_model = Sensor() 
    sensor_model.loadModel(path_loss_file, gp_file)
    references = np.array(readData(result_folder+"averaged_coordinates.csv", True, ' '))[:, 0:2]
    
    # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(references[:,0])
    x_max = np.max(references[:,0])
    y_min = np.min(references[:,1])
    y_max = np.max(references[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
    
        # Calculate probability for each point
        best_point, probability_list, _ = modelPredict(sensor_model, points, X[random_index], Z[random_index], True)
        
        # Plot 
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        visualizeResults(X_grid, Y_grid, probability_list, X[random_index], best_point, references)

def modelPredict(model, points, position, observation, verbose=True, mask = None):
    """
    Predict probability of getting observation at given position by using perceptual model
    Args:
        model: object, perceptual model
        points: n x 2 numpy array, points representing the plane
        position: (2,) or (2,1)/(1,2) numpy array, position
        observation: m x 1 numpy array, RSSI observation
        verbose: boolean, to print the details 
        mask: n x 1 binary numpy array, used to filter out impossible points
    
    Returns:
        best_point: (2, 1) or (2,), point with highted probability prediction
        probability_list: n x 1, numpy array
        difference: float, euclidian distance between actual position and estimated position
    """    
    probability_list  = model.predict(points, observation, verbose)
    if mask is not None:
        probability_list = probability_list * mask
    best_point = points[np.argmax(probability_list)]
    difference = np.sum((best_point - position)**2)**0.5
    if verbose:
        print("Difference: ", difference)
    return best_point, probability_list, difference
        
def visualizeResults(X_grid, Y_grid, probability_list, test_point, best_point, references):
    """
    Plot the result
    Args:
        X_grid: n x m numpy array, x coordinates of planar points
        Y_grid: n x m numpy array, y coordinates of planar points
        probability_list: n x m numpy array, probabilities evaluated at planar points
        test_point: d x 1 numpy array, RSSI observation
        best_point: boolean, to print the details 
        references: k x 2 numpy array, reference points used to train model
    """   
    ax = plt.axes(projection='3d')
    ax.plot_surface(X_grid, Y_grid, probability_list, label='Distribution')
    ax.scatter3D(test_point[0], test_point[1], np.max(probability_list)*1.1, color="limegreen", label='Test point')
    ax.scatter3D(references[:, 0], references[:, 1], 2*np.min(probability_list), color='red', label = "Reference points")
    ax.scatter3D(best_point[0], best_point[1], np.max(probability_list)*1.1, color='yellow', label = "Calcualted point")
    ax.set_xscale('linear')
    ax.set_yscale('linear')
    plt.show()
        
def testKNNPredictor(X, Z, coordinates_file, rss_file):
    """
    Test fucntion for testing KNN predictor
    Args:
        X: n x 2 numpy array, position data
        Z: n x m numpy array, normalized rss data
        coordinates_file: string, file to load reference coordinates data
        rss_file: string, file to load reference rss data
    """
    # Initialize KNN 
    knn = KNN()
    knn.loadData(coordinates_file, rss_file)
    references = np.array(readData(coordinates_file, True, ' '))[:, 0:2]
    
    # Choose a random point 
    N = X.shape[0]
    
   # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(references[:,0])
    x_max = np.max(references[:,0])
    y_min = np.min(references[:,1])
    y_max = np.max(references[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
    
        # Calculate probability for each point
        best_point, probability_list, _ = modelPredict(knn, points, X[random_index], Z[random_index], True)
        
        # Plot 
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        visualizeResults(X_grid, Y_grid, probability_list, X[random_index], best_point, references)
        
def testHybridPredictor(X, Z, coordinates_file, rss_file, path_loss_file, gp_file):
    """
    Test fucntion for testing sensor model training
    Args:
        X: n x 2 numpy array, position data
        Z: n x m numpy array, normalized rss data
        coordinates_file: string, file to load reference coordinates data
        rss_file: string, file to load reference rss data
        path_loss_file: string, file to load trained path loss model
        gp_file: string, file to load trained GP model
    """
    # Initialize hbd 
    hbd = Hybrid()
    hbd.loadModel(coordinates_file, rss_file, path_loss_file, gp_file)
    references = np.array(readData(coordinates_file, True, ' '))[:, 0:2]
    
    # Choose a random point 
    N = X.shape[0]
    
   # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(references[:,0])
    x_max = np.max(references[:,0])
    y_min = np.min(references[:,1])
    y_max = np.max(references[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
    
        # Calculate probability for each point
        best_point, probability_list, _ = modelPredict(hbd, points, X[random_index], Z[random_index], True)

        # Plot 
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        visualizeResults(X_grid, Y_grid, probability_list, X[random_index], best_point, references)
        
def modelTest(model_option=True, visualize_option=False):
    # Get evaluate folders
    parent_folder = "/home/andylei/catkin/src/thesis/data/evaluate"
    subfolder_list = ["/four/", "/seven/", "/ten/", "/fourteen/", "/25/", "/50/"]
    
    # Load test coordinates
    X = np.array(readData(parent_folder + "/test_coordinates.csv", True, ' '))[:, 0:2]
    
    # Draw test coordinates
    N = X.shape[0]
    number_test_points = int(N/1)
    indexes = np.random.randint(0, N, size = number_test_points)
    
    # Plane points
    num_x = 100
    num_y = 40
    X_grid, Y_grid, points = generatePointsOverPlane(-25, 0, -4, 5, num_x, num_y)
    
    for subfolder in subfolder_list:
        # Load model
        folder = parent_folder + subfolder
        if model_option:
            sensor_model = Sensor() 
            sensor_model.loadModel(folder+"pathLoss.csv", folder + "GP.zip")
        else:
            sensor_model = KNN() 
            sensor_model.loadData(folder+"training_coordinates.csv", folder + "training_data.csv")
        
        # Load reference points
        references = np.array(readData("/home/andylei/catkin/src/thesis/data/training/raw" + "/coordinates.csv", True, ' '))[:, 0:2]
        Z = np.array(readData(folder + "test_data.csv", True))
        
        # Compute difference
        difference_list = np.zeros(number_test_points)
        for i in range(number_test_points):        
            # Predict
            index = indexes[i]
            best_point, probability_list, difference = modelPredict(sensor_model, points, X[index], Z[index], False)
            difference_list[i] = difference
            
            # Reshape
            probability_list = probability_list.reshape(num_y, num_x)
            
            # Visualize probability
            if difference > 3 and visualize_option:
                visualizeResults(X_grid, Y_grid, probability_list, X[index], best_point, references)
            
            
        print(subfolder)
        print("Mean: ", np.mean(difference_list))
        print("Variance: ", np.var(difference_list))
        print("Minimum: ", np.min(difference_list))
        print("Maximum: ", np.max(difference_list))
    
def modelTestReference(model_option = True, visualize_option = False):
    # Get evaluate folders
    parent_folder = "/home/andylei/catkin/src/thesis/data/development"
    
    # Load training data
    training = np.array(readData(parent_folder + "/meanTraining.csv", True, ','))
    training_X = training[:, 0:2]
    training_Z = normalizeRss(training[:,2:13], -100)
    
    # Load test data
    test = np.array(readData(parent_folder + "/meanTest.csv", True, ','))
    test_X = test[:, 0:2]
    test_Z = normalizeRss(test[:,2:13], -100) 
       
    # Draw test coordinates
    N = test_X.shape[0]
    number_test_points = int(N/3)
    indexes = np.random.randint(0, N, size = number_test_points)
    
    # Plane points
    num_x = 50
    num_y = 50
    X_grid, Y_grid, points = generatePointsOverPlane(np.min(training_X[:,0]) - 1, np.max(training_X[:,0]) + 1, np.min(training_X[:,1]) - 1, np.max(training_X[:,1]) + 1, num_x, num_y)
    tree = KDTree(training_X)
    distance_vector, _ =  tree.query(points, k=1)
    mask = np.array(distance_vector).reshape(-1) < 0.5
    
    # Various tests
    tests = [4, 6, 8, 11]
    for test in tests:
        print("Test " + str(test))
        
        # Load model
        if model_option:
            sensor_model = Sensor() 
            sensor_model.setData(training_X, training_Z[:,0:test])
            sensor_model.optimize()
        else:
            sensor_model = KNN() 
            sensor_model.setData(training_X, training_Z[:,0:test])
        
        Z = test_Z[:, 0:test]
        
        # Compute difference
        difference_list = np.zeros(number_test_points)
        for i in range(number_test_points):        
            # Predict
            index = indexes[i]
            best_point, probability_list, difference = modelPredict(sensor_model, points, test_X[index], Z[index], False, mask)
            difference_list[i] = difference
            
            # Reshape
            probability_list = probability_list.reshape(num_y, num_x)
            
            # Visualize probability
            if visualize_option:
                visualizeResults(X_grid, Y_grid, probability_list, test_X[index], best_point, training_X)
            
        
        print("Mean: ", np.mean(difference_list))
        print("Variance: ", np.var(difference_list))
        print("Minimum: ", np.min(difference_list))
        print("Maximum: ", np.max(difference_list))
      
if __name__ == "__main__":
    datafolder = "/home/andylei/catkin/src/thesis/data/test/"
    result_folder = "/home/andylei/catkin/src/thesis/result/"
    test_coordinates = np.array(readData(datafolder + "raw/coordinates.csv", True, ' '))[:, 0:2]
    test_data = np.array(readData(datafolder + "result/data.csv", True))
    training_coordinates = np.array(readData(result_folder + "averaged_coordinates.csv", True, ' '))[:, 0:2]
    if False:
        compareClouds(test_coordinates, training_coordinates, "test", "training")
    
    # testSensorModel(test_coordinates, test_data, result_folder+"pathLoss.csv", result_folder+"GP")
    # testModelSaveAndLoad(test_coordinates, test_data, result_folder+"pathLoss.csv", result_folder+"GP.zip")
    # testModelPerformance(test_coordinates, test_data, result_folder+"pathLoss.csv", result_folder+"GP.zip")
    # testKNNPredictor(test_coordinates, test_data, result_folder + "averaged_coordinates.csv", result_folder + "averaged_data.csv")
    # testHybridPredictor(test_coordinates, test_data, result_folder + "averaged_coordinates.csv", result_folder + "averaged_data.csv", result_folder+"pathLoss.csv", result_folder+"GP.zip")
    modelTestReference(False, False)
    # modelTest(False, False)