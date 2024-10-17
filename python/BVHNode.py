import numpy as np
from CHANNEL import CHANNEL
from Helper import R_x, R_y, R_z

class BVHNode:

    def __init__(self, name, parent):
        self._mParent = parent
        self._mName = name
        self._mChannelOffset = 0
        self._mNumChannels = 0

        self._mR = np.full(3, 0)
        self._mChannel = []
        self._mChildren = []
    

    def SetChannel(self, c_offset, c_name)
        self._mChannelOffset = c_offset
        self._mNumChannels = c_name.size
        for cn in c_name:
            self._mChannel.append(CHANNEL.CHANNEL_NAME[cn])
        
    def Set(self, m_t):
        self._mR.setIdentity()
        for i in range(self._mNumChannels):
            if self._mChannel[i] == CHANNEL.Xrot:
                self._mR = self._mR * R_x(m_t[self._mChannelOffset + i])
            elif self._mChannel[i] == CHANNEL.Yrot:
                self._mR = self._mR * R_y(m_t[self._mChannelOffset + i])
            elif self._mChannel[i] == CHANNEL.Zrot:
                self._mR = self._mR * R_z(m_t[self._mChannelOffset + i])
            else:
                pass
    
    def Set(self, R_t):
        self._mR = R_t

    def Get(self):
        return self._mR
    
    def AddChild(self, child):
        self._mChildren.append(child)
    
    def GetNode(self, name):
        if self._mName == name:
            return self
        for child in self._mChildren:
            bn = child.GetNode(name)
            if bn != None:
                return bn
        return None
    

    
