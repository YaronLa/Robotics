"""
Created on Tue Sep 13 13:45:28 2022
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import time

plt.close("all")


def gauss(x, mu, sigma):
    val = ( 1/( np.sqrt(2*np.pi) *sigma ) ) * np.exp( ((-1/2)*( (x-mu)**2.0 )/ sigma**2.0 ) )
    return val

def uniform():
    return 1/15

def p(x):
    p1 = gauss(x,2.0,1.0)
    p2 = gauss(x,5.0,2.0)
    p3 = gauss(x,9.0,1.0)
    pose = 0.3*p1 + 0.4*p2 + 0.3*p3
    return pose

def initproposal(k):
    poses = np.random.uniform(0,15, k)
    return np.array(poses)

def calcWeight(poses):
    weights = p(poses)/uniform()
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
start_time = time.perf_counter()
result = particle()
end_time = time.perf_counter()
print(f'{(end_time-start_time):.5f}')


#print(len(result))


nej = initproposal(k)
plt.close("all")
plt.figure()
plt.xlabel("Pose")
plt.ylabel("Counts")
plt.title(f"K = {k}")
plt.hist(result, bins =int(math.sqrt(k)))
plt.plot(nej,p(nej)*0.1*k, ".")
plt.show()































