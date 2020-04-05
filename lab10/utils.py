import numpy as np


def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)


def dist(p0, p1):
    return np.sqrt(np.sum((p1 - p0) ** 2))
