import numpy as np
import pyMyIsometry3d
from Anchor import Anchor
from Helper import affine3d

class Muscle:
    def __init__(self, name, f0, lm0, lt0, pen_angle, lmax):
        self._l_mt0 = 1.0
        self._l_mt = 1.0
        self._activation = 0.0
        self._f_toe = 0.33 
        self._k_toe = 3.0
        self._k_lin = 51.878788
        self._e_toe = 0.02
        self._e_t0 = 0.033
        self._k_pe = 4.0
        self._e_mo = 0.6
        self._gamma = 0.5

        self._name = name
        self._f0 = f0
        self._l_m0 = lm0
        self._l_t0 = lt0
        self._l_m = self._l_mt - self._l_t0
        self._l_mt_max = lmax

        self._mAnchors = list()
        self._num_related_dofs = 0
        self._related_dof_indices = []
        self._mCachedAnchorPositions = []
        self._mCachedJs = None

        
    def AddAnchor5(self, skel, bn, glob_pos, num_related_bodies):
        distance = np.full(skel.getNumBodyNodes(), 0.0)
        local_positions = np.full((3, skel.getNumBodyNodes()), None)

        for i in range(skel.getNumBodyNodes()):
            T = skel.getBodyNode_i_getTransform_mul_getParentJoint_getTransformFromChildBodyNode(i)
            #T = pyMyIsometry3d.pyMyIsometry3d(skel.getBodyNode_i_getTransform(i)).to_matrix() @ pyMyIsometry3d.MyIsometry3d().to_matrix()
            local_positions[:, i] = affine3d(skel.getBodyNode_i_getTransform_inverse_to_matrix(i), glob_pos)
            distance[i] = np.linalg.norm(glob_pos - T[0:3, 3])
        
        index_sort_by_distance = np.argsort(distance)

        lbs_body_nodes = list()
        lbs_local_positions = list()
        lbs_weights = list()

        total_weight = 0.0

        if distance[index_sort_by_distance[0]] < 0.08:
            lbs_weights.append(1.0 / np.sqrt(distance[index_sort_by_distance[0]]))
            total_weight += lbs_weights[0]
            lbs_body_nodes.append(skel.getBodyNode_i(index_sort_by_distance[0]))
            lbs_local_positions.append(local_positions[:, index_sort_by_distance[0]])

            tmp = lbs_body_nodes[0].getParentBodyNode()
            if not lbs_body_nodes[0].getParentBodyNode().isNull():
                bn_parent = lbs_body_nodes[0].getParentBodyNode()
                lbs_weights.append(1.0 / np.sqrt(distance[bn_parent.getIndexInSkeleton()]))
                total_weight += lbs_weights[1]
                lbs_body_nodes.append(bn_parent)
                lbs_local_positions.append(local_positions[:, bn_parent.getIndexInSkeleton()])
            
        else:
            total_weight = 1.0
            lbs_weights.append(1.0)
            lbs_body_nodes.append(bn)
            lbs_local_positions.append(affine3d(bn.getTransform_inverse(),  glob_pos))

        for i in range(len(lbs_body_nodes)):
            lbs_weights[i] /= total_weight
    
        self._mAnchors.append(Anchor(lbs_body_nodes, lbs_local_positions, lbs_weights))

        n = len(self._mAnchors)
        if n > 1:
            self._l_mt0 += np.linalg.norm(self._mAnchors[n - 1].GetPoint() - self._mAnchors[n - 2].GetPoint())
        
        self._mCachedAnchorPositions = np.full((3, n), 0)
        self.Update()

        Jt = self.GetJacobianTranspose()
        Ap, p = self.GetForceJacobianAndPassive()
        JtA = Jt @ Ap
        self._num_related_dofs = 0
        self._related_dof_indices.clear()

        for i in range(JtA.shape[0]):
            if np.abs(JtA[i]) > 1e-3:
                self._num_related_dofs += 1
                self._related_dof_indices.append(i)


    def AddAnchor3(self, bn, glob_pos):
        lbs_body_nodes = list()
        lbs_local_positions = list()
        lbs_weights = list()

        lbs_body_nodes.append(bn)
        transform_inv = bn.getTransform_inverse()


        lbs_local_positions.append(
                transform_inv[0:3, 0:3] @ glob_pos + transform_inv[0:3, 3])
        lbs_weights.append(1.0)
        
        self._mAnchors.append(Anchor(lbs_body_nodes, lbs_local_positions, lbs_weights))

        n = len(self._mAnchors)
        if n > 1:
            self._l_mt0 += np.linalg.norm(self._mAnchors[n - 1].GetPoint() - self._mAnchors[n - 2].GetPoint())
        
        self._mCachedAnchorPositions = np.full((3, n), 0)
        self.Update()

        Jt = self.GetJacobianTranspose()
        Ap, p = self.GetForceJacobianAndPassive()
        JtA = Jt @ Ap
        self._num_related_dofs = 0
        self._related_dof_indices.clear()

        for i in range(JtA.shape[0]):
            if np.abs(JtA[i]) > 1e-3:
                self._num_related_dofs += 1
                self._related_dof_indices.append(i)

    def ApplyForceToBody(self):
        f = self.GetForce()

        for i in range(len(self._mAnchors) - 1):
            dir = self._mCachedAnchorPositions[:,i + 1] - self._mCachedAnchorPositions[:, i]
            dir = dir / (np.linalg.norm(dir) + 1e-3)
            dir = f * dir
            self._mAnchors[i].bodynodes(0).addExtForce(dir, self._mCachedAnchorPositions[:, i], False, False)

        for i in range(1, len(self._mAnchors)):
            dir = self._mCachedAnchorPositions[:, i - 1] - self._mCachedAnchorPositions[:, i]
            dir = dir / (np.linalg.norm(dir) + 1e-3)
            dir = f * dir
            self._mAnchors[i].bodynodes(0).addExtForce(dir, self._mCachedAnchorPositions[:, i], False, False)
        
    def Update(self):
        for i in range(len(self._mAnchors)):
            p = self._mAnchors[i].GetPoint()
            self._mCachedAnchorPositions[:, i] = self._mAnchors[i].GetPoint()
        
        self._l_mt = self.Getl_mt()
        self._l_mt = self._l_mt - self._l_t0
    
    def GetForce(self):
        return self.Getf_A() * self._activation + self.Getf_p()
    
    def Getf_A(self):
        return self._f0 * self.g_al(self._l_m / self._l_m0)
    
    def Getf_p(self):
        return self._f0 * self.g_pl(self._l_m / self._l_m0)
    
    def Getl_mt(self):
        self._l_mt = 0.0
        for i in range(1, len(self._mAnchors)):
            self._l_mt += np.linalg.norm(self._mCachedAnchorPositions[:, i] - self._mCachedAnchorPositions[:, i - 1])
        return self._l_mt / self._l_mt0
    
    def GetRelatedJtA(self):
        Jt = self.GetJacobianTranspose()
        A, p = self.GetForceJacobianAndPassive()
        JtA = Jt * A
        JtA_reduced = np.full(self._num_related_dofs, 0)
        for i in range(self._num_related_dofs):
            JtA_reduced[i] = JtA(self._related_dof_indices(i))  
        
        return JtA_reduced
    
    def GetJacobianTranspose(self):
        skel = self._mAnchors[0].bodynodes(0).getSkeleton()
        dof = skel.getNumDofs()
        Jt = np.full((dof, 3 * len(self._mAnchors)), 0)

        for i in range(len(self._mAnchors)):
            tm_inv = self._mAnchors[i].bodynodes(0).getTransform_inverse()
            #Jt[0 : dof][i * 3 : i * 3 + 3] = skel.getLinearJacobian(self._mAnchors[i].bodynodes(0), 
            #                                                      tm_inv[0:3, 0:3] @ self._mCachedAnchorPositions[:, i] + tm_inv[0:3, 3]).transpose()   
            tmp1 = skel.getLinearJacobian(self._mAnchors[i].bodynodes(0), tm_inv[0:3, 0:3] @ self._mCachedAnchorPositions[:, i] + tm_inv[0:3, 3])
            tmp2 = tmp1.transpose()
            Jt[0 : dof, i * 3 : i * 3 + 3] = tmp2
        return Jt
            
    def GetForceJacobianAndPassive(self):
        f_a = self.Getf_A()
        f_p = self.Getf_p()

        force_dir = list()
        for i in range(len(self._mAnchors)):
            force_dir.append(np.full(3, 0.0, dtype=np.float64))
        for i in range(len(self._mAnchors) - 1):
            dir = self._mCachedAnchorPositions[:, i+1] - self._mCachedAnchorPositions[:, i]
            dir = dir / (np.linalg.norm(dir) + 1e-3)
            force_dir[i] += dir
        
        for i in range(1, len(self._mAnchors)):
            dir = self._mCachedAnchorPositions[:, i-1] - self._mCachedAnchorPositions[:, i]
            dir = dir / (np.linalg.norm(dir) + 1e-3)
            force_dir[i] += dir
        
        A = np.full(3 * len(self._mAnchors), 0)
        p = np.full(3 * len(self._mAnchors), 0)
        for i in range(len(self._mAnchors)):
            A[i * 3 : i * 3 + 3] = f_a * force_dir[i]
            p[i * 3 : i * 3 + 3] = f_p * force_dir[i]
        
        return A, p
    
    def GetRelatedJoints(self):
        skel = self._mAnchors[0].bodynodes(0).getSkeleton()
        jns = {}
        jns_related = []
        for i in range(skel.getNumJoints()):
            jns[skel.getJoint(i)] = 0
        
        dl_dtheta = self.Getdl_dtheta()

        for i in range(dl_dtheta.rows()):
            if np.abs(dl_dtheta(i)) > 1e-6:
                jns[skel.getDof(i).getJoint()] += 1
        
        for jn, s in jns:
            if s > 0:
                jns_related.append(jn)  
        return jns_related
    
    def GetRelatedBodyNodes(self):
        bns_related = list()
        rjs = self.GetRelatedJoints()
        for joint in rjs:
            bns_related.append(joint.getChildBodyNode())

        return bns_related
    
    def ComputeJacobians(self):
        skel = self._mAnchors[0].bodynodes(0).getSkeleton()
        dof = skel.getNumDofs()
        self._mCachedJs = np.full((len(self._mAnchors), 3, skel.getNumDofs()), 0)
        for i in range(len(self._mAnchors)):
            for j in range(self._mAnchors(i).num_related_bodies):
                self._mCachedJs[i] += self._mAnchors(i).weights(j) @ skel.getLinearJacobian(self._mAnchors(i).bodynodes(j), 
                                                                                              self._mAnchors(i).local_positions(j))

    def Getdl_dtheta(self):
        self.ComputeJacobians()
        skel = self._mAnchors[0].bodynodes(0).getSkeleton()
        dl_dtheta = np.full(skel.getNumDofs(), 0)
        for i in range(len(self._mAnchors) - 1):
            pi = self._mCachedAnchorPositions[:, i + 1] - self._mCachedAnchorPositions[:, i]
            dpi_dtheta = self._mCachedJs[i + 1] - self._mCachedJs[i]
            dli_d_theta = dpi_dtheta.transpose() * pi / (self._l_mt0 * pi.norm())
            dl_dtheta += dli_d_theta
        
        for i in range(dl_dtheta.shape[0]):
            if np.abs(dl_dtheta[i]) < 1e-6:
                dl_dtheta[i] = 0
        
        return dl_dtheta
    
    def g(self, l_m_):
        e_t = (self._l_mt - l_m_ - self._l_t0) / self._l_t0
        l_m_ = l_m_ / self._l_m0
        f = self.g_t(e_t) - (self.g_pl(l_m_) + self._activation * self.g_al(l_m_))
        return f
    
    def g_t(self, e_t):
        if e_t < self._e_t0:
            f_t = self._f_toe / (np.exp(self._k_toe) - 1) * (np.exp(self._ktoe * e_t / self._e_toe) - 1)
        else:
            f_t = self._k_lin * (e_t  - self._e_toe) + self._f_toe
        return f_t
    
    def g_pl(self, l_m_):
        if l_m_ < 1:
            f_pl = 0
        else:
            f_pl = (np.exp(self._k_pe * (l_m_ - 1) / self._e_mo) - 1) / (np.exp(self._k_pe) - 1)
        return f_pl
    
    def g_al(self, l_m_):
        return np.exp(-(l_m_ - 1.0) * (l_m_ - 1.0) / self._gamma)

    def GetNumRelatedDofs(self):
        return self._num_related_dofs

    def setActivation(self, act):
        self._activation = act
    
    
            




