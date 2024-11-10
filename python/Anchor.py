import numpy as np

class Anchor:

    def __init__(self, bns, lps, ws):
        self._bodynodes = bns
        self._local_positions = lps
        self._weights = ws
        self._num_related_bodies = bns.size()

    def GetPoint(self):
        p = np.full(3, 0)
        for i in range(self._num_related_bodies):
            p += self._weights[i] * (self._bodynodes[i].getTransform() * self._local_positions[i])
        return p
    
    

    
