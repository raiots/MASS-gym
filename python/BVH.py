import numpy as np
from Isometry3d import Isometry3d

class BVH:
    def __init__(self, skel, bvh_map):
        self._mSkeleton = skel
        self._mBVHMap = bvh_map
        self._mCyclic = True

        self._mMotions = []
        self._mMap = {}
        self._mTimeStep = 0
        self._mNumTotalChannels = 0
        self._mNumtotalFrames = 0
        self._mRoot = None
        self._T0 = Isometry3d()
        self._T1 = Isometry3d()

    def GetMotion(self, t):
        k =  t // self._mTimeStep
        if self._mCyclic:
            k = k % self._mNumTotalFrames
        k = np.max(0, np.min(k, self._mNumTotalFrames - 1))
        dt = t / self._mTimeStep - (t // self._mTimeStep)
        m_t = self._mMotions[k]

        for bn_k, bn_v in self._mMap:
            bn_v.Set(m_t) 
        
        dof = self._mSkeleton.getNumDofs()
        p = np.full(dof, 0)

        for ss_k, ss_v in self._mBVHMap:
            bn = self._mSkeleton.getBodyNode(ss_k)
            R = self.Get(ss_v)
            jn = bn.getParentJoint()
            idx = jn.getIndexInSkeleton()

            if jn.getType() == "FreeJoint":
                T = Isometry3d()
                T._translation = 0.01 * m_t[0:3]
                T._linear = R
                p[6 * idx: 6 * idx + 6] = FreeJoint.convertToPositions(T)
            elif jn.getType == "BallJoint": 
                p[3 * idx: 3 * idx + 3] = BallJoint.convertToPositions(R)
            elif jn.getType == "RevoluteJoint":
                u = jn.getAxis()
                aa = BallJoint.convertToPositions(R)
                if np.norm(u - np.array([0, 0, 1])) < 1e-4:
                    val = aa[0]
                elif np.norm(u - np.array([0, 1, 0])) < 1e-4:  
                    val = aa[1]
                else:
                    val = aa[2]
                
                if val > np.pi:
                    val -= 2 * np.pi
                elif val < -np.pi:
                    val += 2 * np.pi
                
                p[idx] = val
        return p
    
    def Get(self, bvh_node):
        return self._mMap[bvh_node].Get()

    def Parse(self, file, cyclic):
        self._mCyclic = cyclic
        pass  # TODO: to use the python's specific file stream handler


    def ReadHierarchy(parent, name, channel_offset, ifs):
        pass # TODO: to use the python's specific file stream handler

