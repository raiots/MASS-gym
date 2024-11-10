from enum import Enum

class CHANNEL(Enum):
    Xpos = 0
    Ypos = 1
    Zpos = 2
    Xrot = 3
    Yrot = 4
    Zrot = 5

    CHANNEL_NAME = 	{"Xposition",Xpos,
                     "XPOSITION",Xpos,
                     "Yposition",Ypos,
                     "YPOSITION",Ypos,
                     "Zposition",Zpos,
                     "ZPOSITION",Zpos,
                     "Xrotation",Xrot,
                     "XROTATION",Xrot,
                     "Yrotation",Yrot,
                     "YROTATION",Yrot,
                     "Zrotation",Zrot,
                     "ZROTATION",Zrot}
    
    

