# pipenv install
# pipenv install numpy

from matplotlib import pyplot as plt
import numpy as np

from airplane_sim.first_airplane import airplane_lrh

print("hello world")

temp = airplane_lrh()

ypoints = np.array([3, 8, 1, 10])

plt.plot(ypoints, linestyle = 'dotted')
plt.show()

