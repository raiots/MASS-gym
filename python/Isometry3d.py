import numpy as np

class Isometry3d:
    def __init__(self):
        self._translation = np.full(3, 0)
        self._linear = np.full((3, 3), 0)


