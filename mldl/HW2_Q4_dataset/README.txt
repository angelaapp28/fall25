##### Problem 4 Multi-Class Linear Classification with Softmax #####

The data points are in 2D, so you can easily visualize them in a 2D plane to help you understand
the distribution of those data points. We have 4 classes in total, which are denoted by 0, 1, 2, 3 in Y for each class.

# TO-DO #

# Use the following reference code to load the training and testing data
import numpy as np

svpath = './Q4_data.npz'
data = np.load(svpath)

# X, array of shape [n_samples, n_features]
# Y, array of shape [n_samples]
X = data['X']
Y = data['Y']
