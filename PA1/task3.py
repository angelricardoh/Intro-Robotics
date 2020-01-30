import numpy as np
import matplotlib.pyplot as plt


def mysum2(A):
    return np.sum(A)


def plotcircle1():
    X = np.sin((np.arange(360) * np.pi / 180.))
    Y = np.cos((np.arange(360) * np.pi / 180.))
    plt.plot(X, Y)
    plt.show()


def plotnorm1():
    mu, sigma = 0, 0.1
    s = np.random.normal(mu, sigma, 10000)
    count, bins, ignored = plt.hist(s, 20, density=True)
    plt.plot(bins, 1 / (sigma * np.sqrt(2 * np.pi)) *
             np.exp(- (bins - mu) ** 2 / (2 * sigma ** 2)),
             linewidth=2, color='black')
    plt.show()


# Test cases 4.2
print(mysum2([1, 5, 7]))
print(mysum2([]))
print(mysum2([-5, 3]))

# Test cases 4.3
plt.figure()  # Open plot in a new window
plotcircle1()

# Test cases 4.4
plt.figure()
plotnorm1()
