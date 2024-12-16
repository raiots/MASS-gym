import numpy as np
import pyMyDARTHelper
import pyMySkeletonPtr
import pyMuscle
import pyBVH
import pyMyIsometry3d
import pyMyBodyNodePtr
import xml.etree.ElementTree as ET

from Isometry3d import Isometry3d

class Character:
    def __init__(self):
        self._mSkeleton = None
        self._mBVH = None
        self._mTC = np.full((4, 4), 0)
        self._mTC[0:3, 0:3] = np.identity(3)

        self._mMuscles = []
        self._mEndEffectors = []
        self._mKp = None
        self._mKv = None
    
    def LoadSkeleton(self, path, create_obj):
        self._mSkeleton = pyMyDARTHelper.BuildFromFile(path, create_obj)
        bvh_map = {}
        tree = ET.parse(path)
        root = tree.getroot()
        skel_elem = root #.find('Skeleton') # finds the first child with a particular tag
        for node in skel_elem.findall('Node'):
            if 'endeffector' in node.attrib:
                ee = node.attrib['endeffector']
                if ee == 'True':
                    self._mEndEffectors.append(self._mSkeleton.getBodyNode(node.attrib['name']))
                
            
            joint_elem = node.find('Joint')
            if 'bvh' in joint_elem.attrib:
                bvh_map[node.attrib['name']] = joint_elem.attrib['bvh']
        
        self._mBVH = pyBVH.pyBVH(self._mSkeleton, bvh_map)
    
    def LoadMuscles(self, path):
        tree = ET.parse(path)
        if tree is None:
            raise 'LoadMuscle from path = {} failed'.format(path)
        
        root = tree.getroot()
        muscledoc = root # .find('Muscle')
        for unit in muscledoc.findall('Unit'):
            name = unit.attrib['name']
            f0 = float(unit.attrib['f0'])
            lm = float(unit.attrib['lm'])
            lt = float(unit.attrib['lt'])
            pa = float(unit.attrib['pen_angle'])
            lmax = float(unit.attrib['lmax'])      

            self._mMuscles.append(pyMuscle.pyMuscle(name,f0,lm,lt,pa,lmax)) 
            all_waypoints = unit.findall('Waypoint')  
            num_waypoints = len(all_waypoints)

            i = 0
            for waypoint in all_waypoints:
                body = waypoint.attrib['body']
                glob_pos = np.fromstring(waypoint.attrib['p'], dtype=np.float64, sep=' ')
                if i == 0 or i == num_waypoints - 1:
                    self._mMuscles[-1].AddAnchor(self._mSkeleton.getBodyNode(body), glob_pos)
                else:
                    self._mMuscles[-1].AddAnchor(self._mSkeleton, self._mSkeleton.getBodyNode(body), glob_pos, 2)
                i += 1

    def LoadBVH(self, path, cyclic):
        if self._mBVH == None:
            raise ("Initialize BVH class first")
        self._mBVH.Parse(path, cyclic)
    
    
    def Reset(self):
        #self._mTC = self._mBVH.GetT0()
        #self._mTC.translation()[1] = 0.0
        self._mTC = self._mBVH.GetT0().to_matrix()
        self._mTC[2, 3] = 0.0 # reset_translation_y() # self._mTC.translation() [1]= 0.0

    
    def SetPDParameters(self, kp, kv):
        dof = self._mSkeleton.getNumDofs()
        self._mKp = np.full(dof, kp)
        self._mKv = np.full(dof, kv)
    
    def GetSPDForces(self, p_desired):
        q = self._mSkeleton.getPositions()
        dq = self._mSkeleton.getVelocities()
        dt = self._mSkeleton.getTimeStep()
        M_inv = np.linalg.inv(self._mSkeleton.getMassMatrix() + np.diag(self._mKv * dt))
        qdqdt = q + dq * dt
        p_diff = -self._mKp * (self._mSkeleton.getPositionDifferences(qdqdt, p_desired))
        v_diff = -self._mKv * (dq)
        ddq = M_inv @ (-self._mSkeleton.getCoriolisAndGravityForces() + p_diff + v_diff + self._mSkeleton.getConstraintForces())
        tau = p_diff + v_diff - dt * self._mKv * (ddq)
        tau[0:6] = 0

        return tau

    def GetTargetPositions(self, t, dt):
        p = self._mBVH.GetMotion(t)
        T_current = pyMyDARTHelper.FreeJoint_convertToTransform(p[0:6])
        T_current = np.linalg.inv(self._mBVH.GetT0().to_matrix()) @ T_current.to_matrix()
        T_head = self._mTC * T_current
        p_head = pyMyDARTHelper.FreeJoint_convertToPositions(T_head)
        p_head[0:6] = p_head

        if self._mBVH.IsCyclic():
            t_mod = np.fmod(t, self._mBVH.GetMaxTime())
            t_mod = t_mod / self._mBVH.GetMaxTime()

            r = 0.95
            if t_mod > r:
                ratio = 1.0 / (r - 1.0) * t_mod - 1.0 / (r - 1.0)
                T01 = self._mBVH.GetT1().to_matrix() @ (np.linalg.inv(self._mBVH.GetT0().to_matrix()))
                delta = T01[1, 3]# T01.translation()[1] # to get y
                delta *= ratio
                p[5] += delta
                
            tdt_mod = np.fmod(t + dt, self._mBVH.GetMaxTime())
            if tdt_mod - dt < 0.0:
                T01 = self._mBVH.GetT1().to_matrix() @ (np.linalg.inv(self._mBVH.GetT0().to_matrix()))
                p01 = np.log(T01[0:3, 0:3]) #np.log(T01.linear) 
                p01[0] = 0.0
                p01[2] = 0.0
                T01[0:3, 0:3] = np.exp(p01)

                self._mTC = T01 * self._mTC
                self._mTC[1, 3] = 0.0 # set y to be 0
            
            return p
    
    def GetTargetPosAndVel(self, t, dt):
        p = self.GetTargetPositions(t, dt)
        Tc = self._mTC
        p1 = self.GetTargetPositions(t + dt, dt)
        self._mTC = Tc
        return p, (p1 - p) / dt
    
    def GetSkeleton(self):
        return self._mSkeleton
    
    def GetNumOfMuscles(self):
        return len(self._mMuscles)

    def getMuscleAt(self, i):
        return self._mMuscles[i]
    
    def hasMuscles(self):
        if len(self._mMuscles) > 0:
            return True
        else:
            return False
    
    def getEndEffectorsAt_i(self, i):
        return self._mEndEffectors[i]
    
    def GetEndEffectors_size(self):
        return len(self._mEndEffectors)
    
    def GetBVH(self):
        return self._mBVH
    
    def GetBVH_GetMaxTime(self):
        return self._mBVH.GetMaxTime()
    
    def GetBVH_GetBVHMap(self):
        return self._mBVH.GetBVHMap()
    
    def GetEndEffectors_i_getCOM(self, i):
        return self._mEndEffectors[i].getCOM()

    
    


