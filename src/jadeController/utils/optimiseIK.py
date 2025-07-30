from jadeController.utils.kukaiiwaIKSolver import FastInverseKinematics
from scipy.optimize import minimize
import numpy as np

def jScore(j, r, l=10):
    if j+1e-6 > r or j-1e-6 < -r:
        return l*l*j*j*j*j
    return (l*j*j*j*j)/(r*r*r*r)

def jointRewardFunction(joints):
    return jScore(joints[0], 170*np.pi/180)+jScore(joints[1], 120*np.pi/180)+jScore(joints[2], 170*np.pi/180)+jScore(joints[3], 120*np.pi/180)+jScore(joints[4], 170*np.pi/180)+jScore(joints[5], 120*np.pi/180)+jScore(joints[6], 175*np.pi/180)

def kuka_pose(x, y, z, a, b, c):
    # Convert angles from degrees to radians
    a_rad = np.deg2rad(a)
    b_rad = np.deg2rad(b)
    c_rad = np.deg2rad(c)
    
    # Rotation about the z-axis by angle a
    Rz = np.array([
        [np.cos(a_rad), -np.sin(a_rad), 0],
        [np.sin(a_rad),  np.cos(a_rad), 0],
        [0,              0,             1]
    ])
    
    # Rotation about the y-axis by angle b
    Ry = np.array([
        [ np.cos(b_rad), 0, np.sin(b_rad)],
        [ 0,             1, 0],
        [-np.sin(b_rad), 0, np.cos(b_rad)]
    ])
    
    # Rotation about the x-axis by angle c
    Rx = np.array([
        [1, 0,             0],
        [0, np.cos(c_rad), -np.sin(c_rad)],
        [0, np.sin(c_rad),  np.cos(c_rad)]
    ])
    
    # Combined rotation: intrinsic rotations applied in sequence: Rz -> Ry -> Rx
    R = Rz @ Ry @ Rx
    
    # Build the homogeneous transformation matrix (4x4)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = [x, y, z]
    
    return T

def optimizeR(pose, rconf):
    def objective(r):
        try:
            joints = FastInverseKinematics(pose, r, rconf)
            return jointRewardFunction(joints)
        except:
            return float('1e+30')
    initial_guess = 0
    result = minimize(objective, initial_guess)
    return result.x[0]

def optimiseRot(x, y, z, rconf):
    def objective(a,b,c,r):
        pose = kuka_pose(x, y, z, a, b, c)
        try:
            joints = FastInverseKinematics(pose, r, rconf)
            return jointRewardFunction(joints)
        except:
            return float('1e+30')
    initial_guess = np.array([0,0,0,0])
    bounds = [(-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi), (-10, 10)]
    result = minimize(lambda params: objective(*params), initial_guess, bounds=bounds)
    return result.x

def optimisePos(a, b, c, rconf):
    def objective(x,y,z,r):
        pose = kuka_pose(x, y, z, a, b, c)
        try:
            joints = FastInverseKinematics(pose, r, rconf)
            return jointRewardFunction(joints)
        except:
            return float('1e+30')
    initial_guess = np.array([0,0,0.5,0])
    bounds = [(-1, 1), (-1, 1), (0.64, 1.36), (-10, 10)]
    result = minimize(lambda params: objective(*params), initial_guess, bounds=bounds)
    return result.x

def optimiseFromXYZ(x, y, z, confs=[0,1,2,3,4,5,6,7]):
    bestScore = float('1e+300')
    bestJoints = None
    for i in confs:
        a, b, c, r = optimiseRot(x, y, z, i)
        pose = kuka_pose(x, y, z, a, b, c)
        try:
            joints = FastInverseKinematics(pose, r, i)
            score = jointRewardFunction(joints)
            if score < bestScore:
                bestScore = score
                bestJoints = joints
        except:
            pass
    return bestJoints

def optimiseFromABC(a, b, c, confs=[0,1,2,3,4,5,6,7]):
    bestScore = float('1e+300')
    bestJoints = None
    for i in confs:
        x, y, z, r = optimisePos(a, b, c, i)
        pose = kuka_pose(x, y, z, a, b, c)
        try:
            joints, _, _ = FastInverseKinematics(pose, r, i)
            score = jointRewardFunction(joints)
            if score < bestScore:
                bestScore = score
                bestJoints = joints
        except:
            pass
    return bestJoints
