from re import sub
from general.utils import *

if __name__ == "__main__":
    folder = "/home/andylei/catkin/src/thesis/data/"
    subfolder_list = []
    for subfolder in subfolder_list:
        # Read data
        data = np.array(readData(folder+subfolder+"data.csv", True))
        ids = readData(folder+subfolder+"selections.csv")[1]
        bins = np.linspace(0.1, 0.8, 100)
        
        # Plot distribution
        for i in range(len(ids)):
            plt.hist(data[:,i], bins, alpha=0.5, label=ids[i])
            plt.legend(loc='upper right')
            plt.title(subfolder)
            plt.show()