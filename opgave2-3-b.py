import numpy as np
import math
import matplotlib.pyplot as plt

"""arlo = robot.Robot()
print("Running...")"""

def gauss(x, mu, sigma):
    val = (1/(np.sqrt(2*np.pi)*sigma))*(np.exp(((-1/2)*((np.array(x)-mu)**2.0)/sigma**2.0 )))
    return val

def p(x):
    p1 = gauss(x,2.0,1.0)
    p2 = gauss(x,5.0,2.0)
    p3 = gauss(x,9.0,1.0)
    pose = 0.3*p1 + 0.4*p2 + 0.3*p3
    return pose

def initproposal(k):
    poses = []
    for i in range(k):
        poses.append(np.random.normal(5,4))
    return np.array(poses)

def uniform():
    return 1/15
"""def proposal():
    poses = []
    poses.append()np.
    return prop"""
def normal(poses):
    return gauss(poses,5,4)

def calcWeight(poses):
    weights = p(poses)/normal(poses)
    weights = weights/np.sum(weights)
    return np.array(weights)

def resample(weights, poses,k):
    new_samples = np.random.choice(poses,k, p = weights)

    return np.array(new_samples)

def particle():

    poses = initproposal(k)
    weights = calcWeight(poses)
    new_samples = resample(weights,poses,k)
    return new_samples
k = 10000
result = particle()
print(len(result))
nej = initproposal(k)
plt.close("all")
plt.figure()
plt.xlabel("Pose")
plt.ylabel("Counts")
plt.title(f"K = {k}")
plt.hist(result, bins =int(math.sqrt(k)))
plt.plot(nej,p(nej)*k*0.15, ".")
plt.show()
