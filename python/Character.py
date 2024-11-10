import numpy as np
from Isometry3d import Isometry3d

class Character:
    def __init__(self):
        self._mSkeleton = None
        self._mBVH = None
        self._mTC = Isometry3d.Identity()

        self._mMuscles = None
        self._mEndEffectors = None
        self._mKp = None
        self._mKv = None
    
    def LoadSkeleton(self, path, create_obj):
        pass # TODO: to use python's xml library instead

    def LoadMuscles(self, path):
        pass # TODO: to use python's xml library instead

    def LoadBVH(self, path, cyclic):
        if self._mBVH == None:
            print("Initialize BVH class first")
            return
        self._mBVH.Parse(path, cyclic)
    
    def Reset(self):
        self._mTC = self._mBVH.GetT0()
        self._mTC.translation[1] = 0.0
    
    def SetPDPrameters(self, kp, kv):
        dof = self._mSkeleton.getNumDofs()
        self._mKp = np.full(dof, kp)
        self._mKv = np.full(dof, kv)
    
    def GetSPDForces(self, p_desired):
        q = self._mSkeleton.getPositions()
        dq = self._mSkeleton.getVelocities()
        dt = self._mSkeleton.getTimeStep()
        M_inv = (self._mSkeleton.getMassMatrix() + np.diag(self._mKv * dt)).inverse()
        qdqdt = q + dq @ dt
        p_diff = -self._mKp.multiply(self._mSkeleton.getPositionDifferences(qdqdt, p_desired))
        v_diff = -self._mKv.multiply(dq)
        ddq = M_inv @(-self._mSkeleton.getCoriolisAndGravityForces() + p_diff + v_diff + self._mSkeleton.getConstraintForces())
        tau = p_diff + v_diff - dt * self._mKv.multiply(ddq)
        tau[0:6] = 0

        return tau

    def GetTargetPositions(self, t, dt):
        p = self._mBVH.GetMotion(t)
        T_current = FreeJoint.convertToTransform(p[0:6])
        T_current = self._mBVH.GetT0().inverse() * T_current
        T_head = self._mTC * T_current
        p_head = FreeJoint.convertToPositions(T_head)
        p_head[0:6] = p_head

        if self._mBVH.IsCyclic():
            t_mod = np.fmod(t, self._mBVH.GetMaxTime())
            t_mod = t_mod / self._mBVH.GetMaxTime()

            r = 0.95
            if t_mod > r:
                ratio = 1.0 / (r - 1.0) * t_mod - 1.0 / (r - 1.0)
                T01 = self._mBVH.GetT1() * (self._mBVH.GetT0().inverse())
                delta = T01.translation[1]
                delta *= ratio
                p[5] += delta
                
            tdt_mod = np.fmod(t + dt, self._mBVH.GetMaxTime())
            if tdt_mod - dt < 0.0:
                T01 = self._mBVH.GetT1() * (self._mBVH.GetT0().inverse())
                p01 = np.log10(T01.linear) # TODO: dart::math::logmap
                p01[0] = 0.0
                p01[2] = 0.0
                T01.linear = np.exp(p01) # TODO: dart::math::expMapRot

                self._mTC = T01 * self._mTC
                self._mTC.translation[1] = 0.0
            
            return p
    
    def GetTargetPosAndVel(self, t, dt):
        p = self.GetTargetPositions(t, dt)
        Tc = self._mTC
        p1 = self.GetTargetPositions(t + dt, dt)
        self._mTC = Tc
        return p, (p1 - p) / dt
    
    


