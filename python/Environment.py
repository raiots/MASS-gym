import numpy as np
from Isometry3d import Isometry3d
import os
import pyMyWorldPtr
from Character import Character
import pyMyDARTHelper
import pyMySkeletonPtr

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
        self._mCurrentMuscleTuple = {}

        self._mSimCount = 0
        self._mRandomSampleIndex = 0

    def Initialize_from_file(self, meta_file, load_obj):
        with open(meta_file) as ifs:
            lines = ifs.readlines()  
            character = Character()
            MASS_ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
            for line in lines:
                args = line.split()
                if args[0] == 'use_muscle':
                    if args[1] == 'true':
                        self.SetUseMuscle(True)
                    elif args[1] == 'false':
                        self.SetUseMuscle(False)
                    else:
                        raise ValueError('use_muscle: true or false')
                elif args[0] == 'con_hz':
                    self.SetControlHz(int(args[1]))
                elif args[0] == 'sim_hz':
                    self.SetSimulationHz(int(args[1]))
                elif args[0] == 'skel_file':                  
                    str2 = args[1]
                    character.LoadSkeleton(os.path.join(MASS_ROOT_DIR, str2), load_obj)
                elif args[0] == 'muscle_file':
                    if self.GetUseMuscle():
                        str2 = args[1]
                        character.LoadMuscles(os.path.join(MASS_ROOT_DIR, str2))
                    else:
                        raise ValueError('muscle file no use muscle')
                elif args[0] == "bvh_file":
                    str2 = args[1]
                    str3 = args[2]
                    cyclic = False
                    if str3 == 'true':
                        cyclic = True
                        character.LoadBVH(os.path.join(MASS_ROOT_DIR, str2), cyclic)
                elif args[0] == 'reward_param':
                    a = float(args[1])
                    b = float(args[2])
                    c = float(args[3])
                    d = float(args[4])
                    self.SetRewardParameters(a,b,c,d)
                else:
                    raise ValueError('no accpeted option')
            # end for
            kp = 300.0
            character.SetPDParameters(kp, np.sqrt(2*kp))
            self.SetCharacter(character)
            self.SetGround(pyMyDARTHelper.BuildFromFile(os.path.join(MASS_ROOT_DIR, "data/ground.xml"), False))
            self.Initialize()
        # end of with

    def Initialize(self):
        if self._mCharacter.GetSkeleton() == None:
            print("Initialize Skeleton first") 
            exit(0)
        
        if self._mCharacter.GetSkeleton().getRootBodyNodeParentJointType() == "FreeJoint":
            self._mRootJointDof = 6
        elif self._mCharacter.GetSkeleton().getRootBodyNodeParentJointType() == "PlanarJoint":
            self._mRootJointDof = 3
        else:
            self._mRootJointDof = 0
        
        self._mNumActiveDof = self._mCharacter.GetSkeleton().getNumDofs() - self._mRootJointDof
        if(self._mUseMuscle):
            num_total_related_dofs = 0
            numMuscles = self._mCharacter.GetNumOfMuscles()
            for i in range(numMuscles):
                m = self._mCharacter.getMuscleAt(i)
                m.Update()
                num_total_related_dofs += m.GetNumRelatedDofs()

            self._mCurrentMuscleTuple['JtA'] = np.full(num_total_related_dofs, 0)
            self._mCurrentMuscleTuple['L'] = np.full(self._mNumActiveDof * self._mCharacter.GetNumOfMuscles(), 0)
            self._mCurrentMuscleTuple['b'] = np.full(self._mNumActiveDof, 0)
            self._mCurrentMuscleTuple['tau_des'] = np.full(self._mNumActiveDof, 0)
            self._mActivationLevels = np.full(self._mCharacter.GetNumOfMuscles(), 0)

        self._mWorld.setGravity(np.array([0, -9.8, 0]))
        self._mWorld.setTimeStep(1.0 / self._mSimulationHz)
        self._mWorld.getConstraintSolverSetDetector()

        self._mWorld.addSkeleton(self._mCharacter.GetSkeleton())
        self._mWorld.addSkeleton(self._mGround)
        self._mAction = np.full(self._mNumActiveDof, 0)

        self.Reset(False)
        self._mNumState = self.GetState().size


    def Reset(self, RSI):
        self._mWorld.reset()
        self._mCharacter.GetSkeleton().clearConstraintImpulses()
        self._mCharacter.GetSkeleton().clearInternalForces()
        self._mCharacter.GetSkeleton().clearExternalForces()

        if RSI:
            t = np.random.uniform(0, self._mCharacter.GetBVH_GetMaxTime() * 0.9)
        else:
            t = 0.0
        
        self._mWorld.setTime(t)
        self._mCharacter.Reset()
        self._mAction.fill(0)

        pv = self._mCharacter.GetTargetPosAndVel(t, 1.0 / self._mControlHz)
        self._mTargetPositions = pv[0]
        self._mTargetVelocities = pv[1]

        self._mCharacter.GetSkeleton().setPositions(self._mTargetPositions)
        self._mCharacter.GetSkeleton().setVelocities(self._mTargetVelocities)   
        self._mCharacter.GetSkeleton().computeForwardKinematics(True, False, False)

    def Step(self):
        if self._mUseMuscle:
            count = 0
            numMuscles = self._mCharacter.GetNumOfMuscles()
            for i in range(numMuscles):
                muscle = self._mCharacter.getMuscleAt(i)
                muscle.setActivation(self._mActivationLevels[count])
                count += 1
                muscle.Update()
                muscle.ApplyForceToBody()

            if self._mSimCount == self._mRandomSampleIndex:
                skel = self._mCharacter.GetSkeleton()
                n = skel.getNumDofs()
                m = self._mCharacter.GetNumOfMuscles() 
                JtA = np.full((n, m), 0, dtype=np.float64)
                Jtp = np.full(n, 0, dtype=np.float64)

                for i in range(m):
                    muscle = self._mCharacter.getMuscleAt(i)
                    Jt = muscle.GetJacobianTranspose()
                    Ap = muscle.GetForceJacobianAndPassive()

                    JtA[0:n, i] = Jt @ Ap[0]
                    Jtp += Jt @ Ap[1]
                
                self._mCurrentMuscleTuple['JtA'] = self.GetMuscleTorques()
                L = JtA[self._mRootJointDof:n, 0:m]
                L_vectorized = np.full((n - self._mRootJointDof) * m, 0)
                for i in range(n - self._mRootJointDof):
                    L_vectorized[i * m : i * m + m] = L[i, 0:m]
                self._mCurrentMuscleTuple['L'] = L_vectorized
                self._mCurrentMuscleTuple['b'] = Jtp[self._mRootJointDof:n]
                self._mCurrentMuscleTuple['tau_des'] = self._mDesiredTorques[self._mDesiredTorques.size - self._mRootJointDof]
                self._mMuscleTuples.append(self._mCurrentMuscleTuple)
            else:
                self.GetMuscleTorques()
                self._mCharacter.GetSkeleton().setForces(self._mDesiredTorques)
            
            self._mWorld.step()
            self._mSimCount += 1
    
    def GetDesiredTorques(self):
        p_des = self._mTargetPositions
        p_des[self._mRootJointDof: self._mTargetPositions.size] += self._mAction
        self._mDesiredTorques = self._mCharacter.GetSPDForces(p_des)
        return self._mDesiredTorques[self._mDesiredTorques.size - self._mRootJointDof]
    
    def GetMuscleTorques(self):
        index = 0
        self._mCurrentMuscleTuple['JtA'].fill(0)
        numMuscles = self._mCharacter.GetNumOfMuscles()
        for i in range(numMuscles):
            muscle = self._mCharacter.getMuscleAt(i)
            muscle.Update()
            JtA_i = muscle.GetRelatedJtA()
            self._mCurrentMuscleTuple['JtA'][index : index + JtA_i.size] = JtA_i
            index += JtA_i.size

        return self._mCurrentMuscleTuple['JtA']
    
    def exp_of_squared(self, vec, w):
        return np.exp(-w * np.sqrt(np.dot(vec, vec)))
    
    def IsEndOfEpisode(self):
        isTerminal = False
        p = self._mCharacter.GetSkeleton().getPositions()
        v = self._mCharacter.GetSkeleton().getVelocities()

        root_y = self._mCharacter.GetSkeleton().getBodyNode0TransformTranslation_y() - self._mGround.getRootBodyNode_getCOM_y()

        isTerminal = False
        if root_y < 1.3:
            isTerminal = True
        elif np.any(np.isnan(p)) or np.any(np.isnan(v)):
            isTerminal = True
        elif self._mWorld.getTime() > 10.0:
            isTerminal = True
        
        return isTerminal
    
    def GetState(self):
        skel = self._mCharacter.GetSkeleton()
        num_body_nodes = skel.getNumBodyNodes()
        p = np.full((num_body_nodes - 1) * 3, 0, dtype=np.float64)
        v = np.full((num_body_nodes) * 3, 0, dtype=np.float64)

        for i in range(1, num_body_nodes):
            p[(i - 1) * 3 : i * 3] = skel.getBodyNode_i_getCOM_root(i)
            v[(i - 1) * 3 : i * 3] = skel.getBodyNode_i_getCOMLinearVelocity(i)
         
        v[-3:] = skel.getBodyNode_0_getCOMLinearVelocity()
        t_phase = self._mCharacter.GetBVH_GetMaxTime()
        phi = np.array([np.fmod(self._mWorld.getTime(), t_phase) / t_phase], dtype=np.float)

        p *= 0.8
        v *= 0.2
        state = np.concatenate((p, v, phi))
        return state
    
    def SetAction(self, a):
        self._mAction = a * 0.1
        t = self._mWorld.getTime()
        pv = self._mCharacter.GetTargetPosAndVel(t, 1.0 / self._mControlHz) 
        self._mTargetPositions = pv[0]
        self._mTargetVelocities = pv[1]

        self._mSimCount = 0
        self._mRandomSampleIndex = np.random.randint(0, self._mSimulationHz / self._mControlHz)
    
    def GetReward(self):
        skel = self._mCharacter.GetSkeleton()
        cur_pos = skel.getPositions()
        cur_vel = skel.getVelocities()

        p_diff_all = skel.getPositionDifferences(self._mTargetPositions, cur_pos)
        v_diff_all = skel.getPositionDifferences(self._mTargetVelocities, cur_vel)

        p_diff = np.zeros(skel.getNumDofs())
        v_diff = np.zeros(skel.getNumDofs())

        bvh_map = self._mCharacter.GetBVH_GetBVHMap()

        for key_ss in bvh_map:
            idx = self._mCharacter.GetSkeleton().getBodyNodeByName_getParentJoint_getIndexInSkeleton(key_ss, 0) 
            joint_type = self._mCharacter.GetSkeleton().getBodyNodeByName_getParentJoint_getType(key_ss)
            if joint_type == "FreeJoint":
                continue
            elif joint_type == "RevoluteJoint":
                p_diff[idx] = p_diff_all[idx]
            elif joint_type == "BallJoint":
                p_diff[idx:idx+3] = p_diff_all[idx:idx+3]
            else:
                continue
        ees_szie = self._mCharacter.GetEndEffectors_size()
        ee_diff = np.full(ees_szie * 3, 0.0)
        for i in range(ees_szie):
            ee_diff[i * 3 : i * 3 + 3] = self._mCharacter.GetEndEffectors_i_getCOM(i)

        com_diff = skel.getCOM()
        
        skel.setPositions(self._mTargetPositions)
        skel.computeForwardKinematics(True, False, False)

        com_diff -= skel.getCOM()
        for i in range(ees_szie):
            ee_diff[i * 3 : i * 3 + 3] -= self._mCharacter.GetEndEffectors_i_getCOM(i) + com_diff

        skel.setPositions(cur_pos)
        skel.computeForwardKinematics(True, False, False)

        r_q = self.exp_of_squared(p_diff, 2.0)
        r_v = self.exp_of_squared(v_diff, 0.1)
        r_ee = self.exp_of_squared(ee_diff, 40.0)
        r_com = self.exp_of_squared(com_diff, 10.0)
        r = r_ee * (self._w_q * r_q + self._w_v * r_v)
        return r

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
        return self._mCurrentMuscleTuple['JtA'].size
    
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

    def GetNumState(self):
        return self._mNumState

    def GetNumAction(self):
        return self._mNumActiveDof
    
    def GetNumSteps(self):
        return self._mSimulationHz / self._mControlHz
    
    def GetActivationLevels(self):
        return self._mActivationLevels

    def GetAverageActivationLevels(self):
        return self._mAverageActivationLevels

    def SetActivationLevels(self, a):
        self._mActivationLevels = a
    
    def GetUseMuscle(self):
        return self._mUseMuscle


        
        
