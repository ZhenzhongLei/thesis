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
    
    # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(X[:,0])
    x_max = np.max(X[:,0])
    y_min = np.min(X[:,1])
    y_max = np.max(X[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
        print("Observation:\n", Z[random_index])
        
        # Calculate probability for each point
        probability_list  = sensor_model.predict(points, Z[random_index], True)
        best_point = points[np.argmax(probability_list)]
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        ax = plt.axes(projection='3d')
        ax.plot_surface(X_grid, Y_grid, probability_list, label='Distribution')
        ax.scatter3D(X[random_index, 0], X[random_index, 1], np.max(probability_list), color="limegreen", label='Test point')
        ax.scatter3D(X[:, 0], X[:, 1], 0.001, color='red', label = "Reference points")
        ax.scatter3D(best_point[0], best_point[1], np.max(probability_list), color='yellow', label = "Calcualted point")
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
    
    # Choose a random point 
    N = X.shape[0]
    
   # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(X[:,0])
    x_max = np.max(X[:,0])
    y_min = np.min(X[:,1])
    y_max = np.max(X[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
        print("Observation:\n", Z[random_index])
        
        # Calculate probability for each point
        probability_list  = knn.predict(points, Z[random_index], True)
        best_point = points[np.argmax(probability_list)]
        print("Difference: ", np.sum((best_point - X[random_index])**2)**0.5)
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        ax = plt.axes(projection='3d')
        ax.plot_surface(X_grid, Y_grid, probability_list, label='Distribution')
        ax.scatter3D(X[random_index, 0], X[random_index, 1], np.max(probability_list) + 0.01, color="limegreen", label='Test point')
        ax.scatter3D(X[:, 0], X[:, 1], 0.001, color='red', label = "Reference points")
        ax.scatter3D(best_point[0], best_point[1], np.max(probability_list) + 0.01, color='yellow', label = "Calcualted point")
        ax.set_xscale('linear')
        ax.set_yscale('linear')
        plt.show()
        
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
    
    # Choose a random point 
    N = X.shape[0]
    
   # Compute probability among the plane
    num_x = 50
    num_y = 50
    x_min = np.min(X[:,0])
    x_max = np.max(X[:,0])
    y_min = np.min(X[:,1])
    y_max = np.max(X[:,1])

    # Generate points among the plane
    X_grid, Y_grid, points = generatePointsOverPlane(x_min, x_max, y_min, y_max, num_x, num_y)
    
    # Compute
    while True:
        random_index = np.random.randint(0, N)
        print("Observation:\n", Z[random_index])
        
        # Calculate probability for each point
        probability_list  = hbd.predict(points, Z[random_index], True)
        best_point = points[np.argmax(probability_list)]
        print("Difference: ", np.sum((best_point - X[random_index])**2)**0.5)
        print("Lowest Probability: ", np.min(probability_list))
        probability_list = probability_list.reshape(num_x, num_y)
        
        # Visualize probability
        ax = plt.axes(projection='3d')
        ax.plot_surface(X_grid, Y_grid, probability_list, label='Distribution')
        ax.scatter3D(X[random_index, 0], X[random_index, 1], np.max(probability_list) + 0.01, color="limegreen", label='Test point')
        ax.scatter3D(X[:, 0], X[:, 1], 0.001, color='red', label = "Reference points")
        ax.scatter3D(best_point[0], best_point[1], np.max(probability_list) + 0.01, color='yellow', label = "Calcualted point")
        ax.set_xscale('linear')
        ax.set_yscale('linear')
        plt.show()
        
if __name__ == "__main__":
    datafolder = "/home/andylei/catkin/src/thesis/data/test/"
    result_folder = "/home/andylei/catkin/src/thesis/result/"
    X = np.array(readData(datafolder + "raw/coordinates.csv", True, ' '))[:, 0:2]
    Z = np.array(readData(datafolder + "result/data.csv", True))
    # testSensorModel(X, Z, result_folder+"pathLoss.csv", result_folder+"GP")
    # testModelSaveAndLoad(X, Z, result_folder+"pathLoss.csv", result_folder+"GP.zip")
    # testModelPerformance(X, Z, result_folder+"pathLoss.csv", result_folder+"GP.zip")
    # testKNNPredictor(X, Z, result_folder + "averaged_coordinates.csv", result_folder + "averaged_data.csv")
    testHybridPredictor(X, Z, result_folder + "averaged_coordinates.csv", result_folder + "averaged_data.csv", result_folder+"pathLoss.csv", result_folder+"GP.zip")