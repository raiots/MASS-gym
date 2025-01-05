import numpy as np
from Environment import Environment
import pyCharacter
import pyMuscle

class EnvManager:
    def __init__(self, meta_file, num_envs, seed=None):
        self._mNumEnvs = num_envs
        self._mEnvs = np.full(num_envs, None)

        if seed is not None:  # dart::math::seedRand();
            np.random.seed(seed)

        # TODO: omp_set_num_threads(mNumEnvs);
        for id in range(num_envs):
            self._mEnvs[id] = Environment('id={}'.format(id))
            self._mEnvs[id].Initialize_from_file(meta_file, False) # TODO: env->Initialize(meta_file,false);
        
        #print('DBG: EnvManager init continue before GetMuscleTorques')
        self._muscle_torque_cols = self._mEnvs[0].GetMuscleTorques().size
        #print('DBG: EnvManager init continue after GetMuscleTorques')
        self._tau_des_cols = self._mEnvs[0].GetDesiredTorques().size 

        self._mEos = np.full(num_envs, None)
        self._mRewards = np.full(num_envs, None)
        self._mStates = np.full((num_envs, self.GetNumState()), 0.0)
        self._mMuscleTorques = np.full((num_envs, self._muscle_torque_cols), None)
        self._mDesiredTorques = np.full((num_envs, self._tau_des_cols), None)

        self._mMuscleTuplesJtA = None
        self._mMuscleTuplesTauDes = None
        self._mMuscleTuplesL = None
        self._mMuscleTuplesb = None
    
        # others, refer to EnvManager(std::string meta_file,int num_envs)

    def GetNumState(self) -> int:
        return self._mEnvs[0].GetNumState()
    

    def GetNumAction(self) -> int:
        return self._mEnvs[0].GetNumAction()
    
    def GetSimulationHz(self) -> int:
        return self._mEnvs[0].GetSimulationHz()
    
    def GetControlHz(self) -> int:
        return self._mEnvs[0].GetControlHz()
    
    def GetNumSteps(self) -> int:
        return self._mEnvs[0].GetNumSteps()
    
    def UseMuscle(self) -> bool:
        return self._mEnvs[0].GetUseMuscle()
    
    def Step(self, id):
        self._mEnvs[id].Step()

    def Reset(self, RSI, id):
        self._mEnvs[id].Reset(RSI)

    def IsEndOfEpisode(self, id) -> bool:
        return self._mEnvs[id].IsEndOfEpisode()

    def GetReward(self, id) -> float:
        return self._mEnvs[id].GetReward()
    
    def Steps(self, num):
        #TODO: pragma omp parallel for
        for id in range(self._mNumEnvs):
            for _ in range(num):
                self.Step(id)
    
    def StepsAtOnce(self):
        num = self.GetNumSteps()
        for id in range(self._mNumEnvs):
            for _ in range(num):
                self.Step(id)

    def Resets(self, RSI):
        for id in range(self._mNumEnvs):
            self._mEnvs[id].Reset(RSI)
    
    def IsEndOfEpisodes(self):
        for id in range(self._mNumEnvs):
            self._mEos[id] = self._mEnvs[id].IsEndOfEpisode()
        return self._mEos
    
    def GetStates(self):
        for id in range(self._mNumEnvs):
            t = self._mEnvs[id].GetState().transpose()
            self._mStates[id] = np.array(t, dtype=np.float64)
        return self._mStates
    
    def SetActions(self, actions):
        for id in range(self._mNumEnvs):
            self._mEnvs[id].SetAction(actions[id].transpose())

    def GetRewards(self):
        for id in range(self._mNumEnvs):
            self._mEnvs[id].PrintInfo('GetRewards-Env{}'.format(id))    
            self._mRewards[id] = self._mEnvs[id].GetReward()

        return self._mRewards
    
    def GetMuscleTorques(self):
        for id in range(self._mNumEnvs):       
            #print('DBG: GetMuscleTorques@EnvManager start->env{}'.format(id), flush=True)   
            self._mMuscleTorques[id] = self._mEnvs[id].GetMuscleTorques()
            #print('DBG: GetMuscleTorques@EnvManager end ->env{}'.format(id), flush=True)   
        return self._mMuscleTorques
    
    def GetDesiredTorques(self):
        for id in range(self._mNumEnvs):
            self._mDesiredTorques[id] = self._mEnvs[id].GetDesiredTorques()
        return self._mDesiredTorques

    def SetActivationLevels(self, activations):
        for id in range(self._mNumEnvs):
            self._mEnvs[id].SetActivationLevels(activations[id])

    
    def ComputeMuscleTuples(self):
        n = 0
        rows_JtA = 0
        rows_tau_des = 0
        rows_L = 0
        rows_b = 0

        for id in range(self._mNumEnvs):
            tps_JtA = self._mEnvs[id].GetMuscleTuples_JtA()
            tps_tau_des = self._mEnvs[id].GetMuscleTuples_tau_des()
            tps_L = self._mEnvs[id].GetMuscleTuples_L()
            tps_b = self._mEnvs[id].GetMuscleTuples_b()
            n_JtA = len(tps_JtA)
            n_tau_des = len(tps_tau_des)
            n_L = len(tps_L)
            n_b = len(tps_b)
            if not (n_JtA == n_tau_des and n_tau_des == n_L and n_L == n_b):
                print('something wrong in ComputeMuscleTuples?')

            n += n_JtA
            if n_JtA > 0:
                rows_JtA = tps_JtA[0].size
                rows_tau_des = tps_tau_des[0].size
                rows_L = tps_L[0].size
                rows_b = tps_b[0].size

        self._mMuscleTuplesJtA = np.full((n, rows_JtA), None)
        self._mMuscleTuplesTauDes = np.full((n, rows_tau_des), None)
        self._mMuscleTuplesL = np.full((n, rows_L), None)
        self._mMuscleTuplesb = np.full((n, rows_b), None)

        o = 0
        for id in range(self._mNumEnvs):
            tps_JtA = self._mEnvs[id].GetMuscleTuples_JtA()
            tps_tau_des = self._mEnvs[id].GetMuscleTuples_tau_des()
            tps_L = self._mEnvs[id].GetMuscleTuples_L()
            tps_b = self._mEnvs[id].GetMuscleTuples_b()
            for i in range(len(tps_JtA)):
                self._mMuscleTuplesJtA[o] = tps_JtA[i]
                self._mMuscleTuplesTauDes[o] = tps_tau_des[i]
                self._mMuscleTuplesL[o] = tps_L[i]
                self._mMuscleTuplesb[o] = tps_b[i]
                o += 1
            tps_JtA.clear()
            tps_tau_des.clear()
            tps_L.clear()
            tps_b.clear()
        
    def GetMuscleTuplesJtA(self):
        return self._mMuscleTuplesJtA
    
    def GetMuscleTuplesTauDes(self):
        return self._mMuscleTuplesTauDes
    
    def GetMuscleTuplesL(self):
        return self._mMuscleTuplesL
    
    def GetMuscleTuplesb(self):
        return self._mMuscleTuplesb
    
    def GetNumTotalMuscleRelatedDofs(self):
        return self._mEnvs[0].GetNumTotalRelatedDofs()
    
    def GetNumMuscles(self):
        num = self._mEnvs[0].GetCharacter().GetNumOfMuscles()
        return num
    
