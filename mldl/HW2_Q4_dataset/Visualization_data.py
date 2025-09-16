import numpy as np
import matplotlib.pyplot as plt

svpath = './Q4_data.npz'
# svpath = './data.npz'
data = np.load(svpath)
print(data.files)
# X, array of shape [n_samples, n_features]
# Y, array of shape [n_samples]
trainX = data['X']
trainY = data['Y']

d=2 # dimension

# add bias
[trainX_n,_]=trainX.shape
trainX=np.hstack((trainX,np.ones((trainX_n,1))))

Fig1=plt.figure(1)
plot_x=trainX[:,0:2]
plot_y=trainY[:]
[n_plot,_]=plot_x.shape

for i in range(n_plot):
    if plot_y[i]==0:
        plt.plot(plot_x[i,0],plot_x[i,1],'bo',markersize=15)
    elif plot_y[i]==1:
        plt.plot(plot_x[i,0],plot_x[i,1],'gx',markersize=15)
    elif plot_y[i]==2:
        plt.plot(plot_x[i,0],plot_x[i,1],'r*',markersize=15)
    else:
        plt.plot(plot_x[i,0],plot_x[i,1],'ks',markersize=15)


plt.show()
