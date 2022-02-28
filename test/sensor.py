from particle.sensor import *


def testPathLossCalculation():
    print("Test path loss calculation:")
    sensor_model = Sensor()
    sensor_model.calculatePathlossValue(np.array([1, 2, 4, 4]).reshape(2,2), np.array([1, 0.5, 2, 3]), 1e-3, True)
    print('\n')
    
def testObjectiveFunction():
    print("Test objective function:")
    X = np.random.random((10, 2))
    Z = np.random.random(10)
    parameters = np.array([1, 0.5, 2, 3])   
    sensor_model = Sensor()
    value = sensor_model.func(parameters, X, Z, sensor_model.epsilon_, sensor_model.penalty_factor_, sensor_model.calculatePathlossValue, True)
    print(value, '\n')

def testSensorModel(data_file, minimum_rss_value, path_loss_file, gp_file):
    # Take in test data
    data = readData(data_file, True, ',')
    data = np.array(data).astype(float)
    
    # Convert to wanted type
    X = data[:, 0:2]
    Z = data[:, 2:13]
    drawDistribution(X)
    
    # Split the data into test and 
    Z = normalizeRss(Z, minimum_rss_value)
    
    # Compute path loss parameters
    sensor_model = Sensor()  
    sensor_model.setData(X, Z)
    sensor_model.initializePathLossParameters()
    sensor_model.calculatePathlossModel()
    compareClouds(X, sensor_model.path_loss_params_[2:4,:].T, "reference postions", "inferred AP positions")
    
    # Calculate GP
    sensor_model.calculateGP()
    
    # Predict 
    random_index = 123 #np.random.randint(0, X.shape[0])
    print("Random drawn data:\n", Z[random_index])
    
    mean, _, probability = sensor_model.predict(X[random_index], Z[random_index])
    print("Predict:\n", mean)
    print("Probability:\n", probability)
    print('\n')
    sensor_model.saveModel(path_loss_file, gp_file)
    
def testModelSaveAndLoad(data_file, minimum_rss_value, path_loss_file, gp_file):
    print('Test saved model')
    data = readData(data_file, True, ',')
    data = np.array(data).astype(float)
    
    # Convert data
    X = data[:, 0:2]
    Z = data[:, 2:13]
    Z = normalizeRss(Z, minimum_rss_value)
    
    # Test
    random_index = 123 #np.random.randint(0, X.shape[0])
    print("Random drawn data:\n", Z[random_index])
    sensor_model = Sensor() 
    sensor_model.loadModel(path_loss_file, gp_file)
    result = sensor_model.predict(X[random_index], Z[random_index])
    print("Result:\n", result)
    
if __name__ == "__main__":
    testPathLossCalculation()
    testObjectiveFunction()
    result_folder = "/home/andylei/catkin/src/thesis/result/"
    testSensorModel("/home/andylei/catkin/src/thesis/data/development/mean.csv", -100, result_folder+"pathLoss.csv", result_folder+"GP")
    testModelSaveAndLoad("/home/andylei/catkin/src/thesis/data/verification/mean.csv", -100, result_folder+"pathLoss.csv", result_folder+"GP.zip")