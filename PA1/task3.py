import numpy as np
import matplotlib.pyplot as plt


def mysum2(A):
    return sum(A)

def plotcircle1():
    X = np.sin((np.arange(360) * np.pi / 180. ))
    Y = np.cos((np.arange(360) * np.pi / 180. ))
#    A = np.sin(np.array((0., 30., 45., 60., 90., 120., 150., 180., 210., 240., 270., 300., 330., 360.)) * np.pi / 180.)
#    B = np.cos(np.array((0., 30., 45., 60., 90., 120., 150., 180., 210., 240., 270., 300., 330., 360.)) * np.pi / 180.)
    plt.plot(X, Y)

def plotnorm1():
    mu, sigma = 0, 0.1
    s = np.random.normal(mu, sigma, 10000)
    count, bins, ignored = plt.hist(s, 20, density=True)
    plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) *
               np.exp( - (bins - mu)**2 / (2 * sigma**2) ),
         linewidth=2, color='black')
    plt.show()


#print(mysum2([1, 5, 7]))
#print(mysum2([]))
#print(mysum2([-5, 3]))

#plotcircle1()

plotnorm1()