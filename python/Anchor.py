import numpy as np
from Helper import affine3d

class Anchor:

    def __init__(self, bns, lps, ws):
        self._bodynodes = bns
        self._local_positions = lps
        self._weights = ws
        self._num_related_bodies = len(bns)
    
    def bodynodes(self, i):
        return self._bodynodes[i]

    def GetPoint(self):
        p = np.full(3, 0.0, dtype=np.float64)
        for i in range(self._num_related_bodies):
            tm = self._bodynodes[i].getTransform()
            x = affine3d(tm, self._local_positions[i])
            y = np.array(x, dtype=np.float64)
            z = float(self._weights[i]) * y
            p[:] += y
        return p
    
    

    
