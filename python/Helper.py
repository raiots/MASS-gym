import numpy as np

def R_x(x):
    cosa = np.cos(x * np.pi / 180.0)
    sina = np.sin(x * np.pi / 180.0)
    R = np.array([[1, 0, 0], 
                  [0, cosa, -sina], 
                  [0, sina, cosa]])
    return R

def R_y(y):
    cosa = np.cos(y * np.pi / 180.0)
    sina = np.sin(y * np.pi / 180.0)
    R = np.array([[cosa, 0, sina], 
                  [0, 1, 0], 
                  [-sina, 0, cosa]])
    return R

def R_z(z):
    cosa = np.cos(z * np.pi / 180.0)
    sina = np.sin(z * np.pi / 180.0)
    R = np.array([[cosa, -sina, 0], 
                  [sina, cosa, 0], 
                  [0, 0, 1]])
    return R

def affine3d(A, x):
    return A[0:3, 0:3] @ x + A[0:3, 3]


