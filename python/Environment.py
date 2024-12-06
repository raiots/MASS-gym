import numpy as np
from Isometry3d import Isometry3d
import os
import pyMyWorldPtr
import pyCharacter
import MyDARTHelper

class Environment:
    def __init__(self):
        self._mControlHz = 30
        self._mSimulationHz = 900
        self._mWorld = pyMyWorldPtr.pyMyWorldPtr()
        self._mUseMuscle = True
        self._w_q = 0.65
        self._w_v = 0.1
        self._w_ee = 0.15
        self._w_com = 0.1

        self._mCharacter = None
        self._mGround = None
        self._mAction = None
        self._mTargetPositions = None
        self._mTargetVelocities = None
        self._mNumState = 0
        self._mNumActiveDof = 0
        self._mRootJointDof = 0

        self._mActivationLevels = None
        self._mAverageActivationLevels = None
        self._mDesiredTorques = None
        self._mMuscleTuples = []
        self._mCurrentMuscleTupe = {}

        self._mSimCount = 0
        self._mRandomSampleIndex = 0

    def Initialize_from_file(self, meta_file, load_obj):
        with open(meta_file) as ifs:
            lines = ifs.readlines()     
            character = pyCharacter.pyCharacter()
            MASS_ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
            for line in lines:
                args = line.split()
                if args[0] is 'use_muscle':
                    if args[1] is 'true':
                        self.SetUseMuscle(True)
                    elif args[1] is 'false':
                        self.SetUseMuscle(False)
                    else:
                        raise ValueError('use_muscle: true or false')
                elif args[0] is 'con_hz':
                    self.SetControlHz(int(args[1]))
                elif args[0] is 'sim_hz':
                    self.SetSimulationHz(int(args[1]))
                elif args[0] is 'skel_file':                  
                    str2 = args[1]
                    character.LoadSkeleton(os.path.join(MASS_ROOT_DIR, str2), load_obj)
                elif args[0] is 'muscle_file':
                    if self.GetUseMuscle():
                        str2 = args[1]
                        character.LoadMuscles(os.path.join(MASS_ROOT_DIR, str2))
                    else:
                        raise ValueError('muscle file no use muscle')
                elif args[0] is "bvh_file":
                    str2 = args[1]
                    str3 = args[2]
                    cyclic = False
                    if str3 is 'true':
                        cyclic = True
                        character.LoadBVH(os.path.join(MASS_ROOT_DIR, str2), cyclic)
                elif args[0] is 'reward_param':
                    a = int(args[1])
                    b = int(args[2])
                    c = int(args[3])
                    d = int(args[4])
                    self.SetRewardParameters(a,b,c,d)
                else:
                    raise ValueError('no accpeted option')
            # end for
            kp = 300.0
            character.SetPDParameters(kp, np.sqrt(2*kp))
            self.SetCharacter(character)
            self.SetGround(MyDARTHelper.BuildFromFile(os.path.join(MASS_ROOT_DIR, "/data/ground.xml")))
            self.Initialize()
        # end of with

    def Initialize(self):
        if self._mCharacter.GetSkeleton() == None:
            print("Initialize Skeleton first")
            exit(0)
        
        if self._mCharacter.GetSkeleton().GetRootBodyNode().getParentJoint().getType() == "FreeJoint":
            self._mRootJointDof = 6
        elif self._mCharacter.GetSkeleton().GetRootBodyNode().getParentJoint().getType() == "PlanarJoint":
            self._mRootJointDof = 3
        else:
            self._mRootJointDof = 0
        
        self._mNumActiveDof = self._mCharacter.GetSkeleton().getNumDofs() - self._mRootJointDof

        if(self._mUseMuscle):
            num_total_related_dofs = 0
            for m in self._mCharacter.GetMuscles():
                m.Update()
                num_total_related_dofs += m.GetNumRelatedDofs()
            
            self._mCurrentMuscleTupe['JtA'] = np.full(num_total_related_dofs, 0)
            self._mCurrentMuscleTupe['L'] = np.full(self._mNumActiveDof * self._mCharacter.GetMuscles().size, 0)
            self._mCurrentMuscleTupe['b'] = np.full(self._mNumActiveDof, 0)
            self._mCurrentMuscleTupe['tau_des'] = np.full(self._mNumActiveDof, 0)
            self._mActivationLevels = np.full(self._mCharacter.GetMuscles().size, 0)

        self._mWorld.setGravity(np.array([0, -9.8, 0]))
        self._mWorld.setTimeStep(1.0 / self._mSimulationHz)
        self._mWorld.getConstraintSolver().setCollisionDetector(BulletCollisionDetector.creat())
        self._mWorld.addSkeleton(self._mCharacter.GetSkeleton())
        self._mWorld.addSkeleton(self._mGround)
        self._mAction = np.full(self._mNumActiveDof, 0)

        self.Reset(False)
        self._mNumState = self.GetState().rows()


    def Reset(self, RSI):
        self._mWorld.reset()
        self._mCharacter.GetSkeleton().clearConstraintImpulses()
        self._mCharacter.GetSkeleton().clearInternalForces()
        self._mCharacter.GetSkeleton().clearExternalForces()

        if RSI:
            t = np.random.uniform(0, self._mCharacter.GetBVH().GetMaxTime() * 0.9)
        else:
            t = 0.0
        
        self._mWorld.setTime(t)
        self._mCharacter.Reset()
        self._mAction.setZero()

        pv = self._mCharacter.GetTargetPositions(t, 1.0 / self._mControlHz)
        self._mTargetPositions = pv[0]
        self._mTargetVelocities = pv[1]

        self._mCharacter.GetSkeleton().setPositions(self._mTargetPositions)
        self._mCharacter.GetSkeleton().setVelocities(self._mTargetVelocities)   
        self._mCharacter.GetSkeleton().computeForwardKinematics(True, False, False)

    def Step(self):
        if self._mUseMuscle:
            count = 0
            for muscle in self._mCharacter.GetMuscles():
                muscle.activation = self._mActivationLevels[count]
                count += 1
                muscle.Update()
                muscle.ApplyForceToBody()

            if self._mSimCount == self._mRandomSampleIndex:
                skel = self._mCharacter.GetSkeleton()
                muscles = self._mCharacter.GetMuscles()

                n = skel.getNumDofs()
                m = muscles.size()
                JtA = np.full((n, m), 0)
                Jtp = np.full(n, 0)

                for i in range(muscles.size()):
                    muscle = muscles[i]
                    Jt = muscle.GetJacobianTranspose()
                    Ap = muscle.GetForceJacobianAndPassive()

                    JtA[0:n, i] = Jt * Ap[0]
                    Jtp += Jt * Ap[1]
                
                self._mCurrentMuscleTupe['JtA'] = self.GetMuscleTorques()
                L = JtA[self._mRootJointDof:n, 0:m]
                L_vectorized = np.full((n - self._mRootJointDof) * m, 0)
                for i in range(n - self._mRootJointDof):
                    L_vectorized[i * m : i * m + m] = L[i, 0:m]
                self._mCurrentMuscleTupe['L'] = L_vectorized
                self._mCurrentMuscleTupe['b'] = Jtp[self._mRootJointDof:n]
                self._mCurrentMuscleTupe['tau_des'] = self._mDesiredTorques[self._mDesiredTorques.rows() - self._mRootJointDof]
                self._mMuscleTuples.append(self._mCurrentMuscleTupe)
            else:
                self.GetMuscleTorques()
                self._mCharacter.GetSkeleton().setForces(self._mDesiredTorques)
            
            self._mWorld.step()
            self._mSimCount += 1
    
    def GetDesiredTorques(self):
        p_des = self._mTargetPositions
        p_des[self._mTargetPositions.rows() - self._mRootJointDof] += self._mAction
        self._mDesiredTorques = self._mCharacter.GetSPDForces(p_des)
        return self._mDesiredTorques[self._mDesiredTorques.rows() - self._mRootJointDof]
    
    def GetMuscleTorques(self):
        index = 0
        self._mCurrentMuscleTupe['JtA'].fill(0)
        for muscle in self._mCharacter.GetMuscles():
            muscle.Update()
            JtA_i = muscle.GetRelatedJtA()
            self._mCurrentMuscleTupe['JtA'][index : index + JtA_i.rows()] = JtA_i
            index += JtA_i.rows()
        
        return self._mCurrentMuscleTupe['JtA']
    
    def exp_of_square(vec, w):
        return np.exp(-w * np.dot(vec, vec))
    
    def IsEndOfEpisode(self):
        isTerminal = False
        p = self._mCharacter.GetSkeleton().getPositions()
        v = self._mCharacter.GetSkeleton().getVelocities()

        root_y = self._mCharacter.GetSkeleton().getBodyNode(0).getTransform().translation[1] - self._mGround.getRootBodyNode().getCOM()[1]

        isTerminal = False
        if root_y < 1.3:
            isTerminal = True
        elif np.isnan(p) or np.isnan(v):
            isTerminal = True
        elif self._mWorld.getTime() > 10.0:
            isTerminal = True
        
        return isTerminal
    
    def GetState(self):
        skel = self._mCharacter.GetSkeleton()
        root = skel.getBodyNode(0)
        num_body_nodes = skel.getNumBodyNodes()
        p = np.full((num_body_nodes - 1) * 3, 0)
        v = np.full((num_body_nodes) * 3, 0)

        for i in range(1, num_body_nodes):
            p[(i - 1) * 3 : i * 3] = skel.getBodyNode(i).getCOM(root)
            v[(i - 1) * 3 : i * 3] = skel.getBodyNode(i).getCOMLinearVelocity()
        
        v[-3:] = root.getCOMLinearVelocity()
        t_phase = self._mCharacter.GetBVH().GetMaxTime()
        phi = np.fmod(self._mWorld.getTime(), t_phase) / t_phase

        p *= 0.8
        v *= 0.2
        state = np.array([p, v, phi])
        return state
    
    def SetAction(self, a):
        self._mAction = a * 0.1
        t = self._mWorld.getTime()
        pv = self._mCharacter.GetTargetPosAndVel(t, 1.0 / self._mControlHz) 
        self._mTargetPositions = pv[0]
        self._mTargetVelocities = pv[1]

        self._mSimCount = 0
        self._mRandomSampleIndex = np.random.randint(0, self._mSimulationHz / self._mControlHz)
        self._mAverageActivationLevels[:] = 0
    
    def GetReward(self):
        skel = self._mCharacter.GetSkeleton()
        cur_pos = skel.getPositions()
        cur_vel = skel.getVelocities()

        p_diff_all = skel.getPositionDifferences(self._mTargetPositions, cur_pos)
        v_diff_all = skel.getPositionDifferences(self._mTargetVelocities, cur_vel)

        p_diff = np.zeros(skel.getNumDofs())
        v_diff = np.zeros(skel.getNumDofs())

        bvh_map = self._mCharacter.GetBVH().GetBVHMap()

        for key_ss, value_ss in bvh_map:
            joint = self._mCharacter.GetSkeleton().getBodyNode(key_ss).getParentJoint()
            idx = joint.getIndexInSkeleton(0)
            if joint.getType() == "FreeJoint":
                continue
            elif joint.getType() == "RevoluteJoint":
                p_diff[idx] = p_diff_all[idex]
            elif joint.getType() == "BallJoint":
                p_diff[idx:idx+3] = p_diff_all[idx:idx+3]
            else:
                continue
        ees = self._mCharacter.GetEndEffectors()
        ee_diff = np.full(ees.size * 3)
        for i in range(ees.size):
            ee_diff[i * 3 : i * 3 + 3] = ees[i].getCOM()

        com_diff = skel.getCOM()

    def SetUseMuscle(self, use_muscle):
        self._mUseMuscle = use_muscle
    
    def SetControlHz(self, con_hz):
        self._mControlHz = con_hz
    
    def SetSimulationHz(self, sim_hz):
        self._mSimulationHz = sim_hz
    
    def SetCharacter(self, character):
        self._mCharacter = character
    
    def SetGround(self, ground):
        self._mGround = ground
    
    def SetRewardParameters(self, w_q, w_v, w_ee, w_com):
        self._w_q = w_q
        self._w_v = w_v
        self._w_ee = w_ee
        self._w_com = w_com

    def GetWorld(self):
        return self._mWorld
    
    def GetCharacter(self):
        return self._mCharacter

    def GetGround(self):
        return self._mGround
    
    def GetControlHz(self):
        return self._mControlHz
    
    def GetSimulationHz(self):
        return self._mSimulationHz
    
    def GetNumTotalRelatedDofs(self):
        return self._mCurrentMuscleTupe['JTA'].size
    
    def GetMuscleTuples_JtA(self):
        tmp = []
        for muscleTupe in self._mMuscleTuples:
            tmp.append(muscleTupe['JtA'])
        return tmp
    
    def GetMuscleTuples_L(self):
        tmp = []
        for muscleTupe in self._mMuscleTuples:
            tmp.append(muscleTupe['L'])
        return tmp

    def GetMuscleTuples_b(self):
        tmp = []
        for muscleTupe in self._mMuscleTuples:
            tmp.append(muscleTupe['b'])
        return tmp
       
    def GetMuscleTuples_tau_des(self):
        tmp = []
        for muscleTupe in self._mMuscleTuples:
            tmp.append(muscleTupe['tau_des'])
        return tmp
    

        
        
