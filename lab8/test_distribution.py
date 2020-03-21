import numpy as np
import matplotlib.pyplot as plt
import lab8_map
from particle_filter import ParticleFilter


# data = np.random.normal(0.5, 0.1, 100)
# plt.hist(data, 20)
# plt.show()

map = lab8_map.Map("lab8_map.json")
particle_filter = ParticleFilter(map, 1)

