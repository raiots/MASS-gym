import numpy as np
from Isometry3d import Isometry3d


def MakeSphereShape(radius):
    return SphereShape(radius)

def MakeBoxShape(size):
    return BoxShape(size)

def MakeCapsuleShape(radius, height):
    return CapsuleShape(radius, height)

def MakeInertia(shape, mass):
    inertial = Inertia()
    inertial.setMass(mass)
    inertial.setMoment(shape.computeInertia(mass))
    return inertial

def MakeFreeJointProperties(name, parent_to_joint, child_to_joint):
    props = FreeJoint.Properties()
    props.mName = name
    props.mT_ParentBodyToJoint = parent_to_joint
    props.mT_ChildBodyToJoint = child_to_joint
    props.mIsPositionLimitEnforced = False
    props.mVelocityLowerLimits = np.full(6, -100.0)
    props.mVelocityUpperLimits = np.full(6, 100.0)
    props.mDampingCoefficients = np.full(6, 0.4)
    return props

def MakePlanarJointProperties(name, parent_to_joint, child_to_joint):
    props = PlanarJoint.Properties()
    props.mName = name
    props.mT_ParentBodyToJoint = parent_to_joint
    props.mT_ChildBodyToJoint = child_to_joint
    props.mIsPositionLimitEnforced = False
    props.mVelocityLowerLimits = np.full(3, -100.0)
    props.mVelocityUpperLimits = np.full(3, 100.0)
    props.mDampingCoefficients = np.full(3, 0.4)
    return props

def MakeBallJointProperties(name, parent_to_joint, child_to_joint, lower, upper):
    props = BallJoint.Properties()
    props.mName = name
    props.mT_ParentBodyToJoint = parent_to_joint
    props.mT_ChildBodyToJoint = child_to_joint
    props.mIsPositionLimitEnforced = False
    props.mPositionLowerLimits = lower
    props.mPositionUpperLimits = upper
    props.mVelocityLowerLimits = np.full(3, -100.0)
    props.mVelocityUpperLimits = np.full(3, 100.0)
    props.mForceLowerLimits = np.full(3, -1000)
    props.mForceUpperLimits = np.full(3, 1000)  
    props.mDampingCoefficients = np.full(3, 0.4)
    return props

def MakeRevoluteJointProperties(name, axis, parent_to_joint, child_to_joint, lower, upper):
    props = RevoluteJoint.Properties()
    props.mName = name
    props.mT_ParentBodyToJoint = parent_to_joint
    props.mT_ChildBodyToJoint = child_to_joint
    props.mIsPositionLimitEnforced = True
    props.mPositionLowerLimits = lower
    props.mPositionUpperLimits = upper
    props.mAxis = axis
    props.mVelocityLowerLimits = np.full(1, -100.0)
    props.mVelocityUpperLimits = np.full(1, 100.0)
    props.mForceLowerLimits = np.full(1, -1000)
    props.mForceUpperLimits = np.full(1, 1000)  
    props.mDampingCoefficients = np.full(1, 0.4)
    return props

def MakeWeldJointProperties(name, parent_to_joint, child_to_joint):
    props = WeldJoint.Properties()
    props.mName = name
    props.mT_ParentBodyToJoint = parent_to_joint
    props.mT_ChildBodyToJoint = child_to_joint
    return props

def MakeBodyNode(skeleton, parent, joint_properties, joint_type, inertia):
    if joint_type == "Free":
        prop = FreeJoint.Properties(joint_properties)
        bn = skeleton.createJointAndBodyNodePair[FreeJoint](parent, prop, BodyNode.AspectProperties(joint_properties.mName)).value
    elif joint_type == "Planar":
        prop = PlanarJoint.Properties(joint_properties)
        bn = skeleton.createJointAndBodyNodePair[PlanarJoint](parent, prop, BodyNode.AspectProperties(joint_properties.mName)).value      
    elif joint_type == "Ball":
        prop = BallJoint.Properties(joint_properties)
        bn = skeleton.createJointAndBodyNodePair[BallJoint](parent, prop, BodyNode.AspectProperties(joint_properties.mName)).value     
    elif joint_type == "Revolute":
        prop = RevoluteJoint.Properties(joint_properties)
        bn = skeleton.createJointAndBodyNodePair[RevoluteJoint](parent, prop, BodyNode.AspectProperties(joint_properties.mName)).value  
    elif joint_tupe == "Weld":     
        prop = WeldJoint.Properties(joint_properties)
        bn = skeleton.createJointAndBodyNodePair[WeldJoint](parent, prop, BodyNode.AspectProperties(joint_properties.mName)).value  
    else:
        raise ValueError("Joint type not supported.")
    bn.setInertia(inertia)
    return bn

def Proj(u, v):
    proj = u.dot(v)/u.dot(u) * u
    return proj

def Orthonormalize(T_old):
    T = Isometry3d()
    T._translation = T_old._translation
    v0 = T_old._linear[:, 0]
    v1 = T_old._linear[:, 1]
    v2 = T_old._linear[:, 2]

    u0 = v0
    u1 = v1 - Proj(u0, v1)  
    u2 = v2 - Proj(u0, v2) - Proj(u1, v2)

    u0.normalize()
    u1.normalize()
    u2.normalize()

    T._linear[:, 0] = u0
    T._linear[:, 1] = u1
    T._linear[:, 2] = u2

    return T

def split_to_double(input, num):
    result = np.fromstring(input, count=num)
    return result

def string_to_vector1d(input):
    return np.fromstring(input, count=1)

def string_to_vector3d(input):
    return np.fromstring(input, count=3)

def string_to_vector4d(input):
    return np.fromstring(input, count=4)

def string_to_vectorXd(input, n):
    return split_to_double(input, n)

def string_to_matrix3d(input):
    return np.fromstring(input, count=9).reshape((3, 3))

def BuildFromFile(path, create_obj):
    pass  # TODO: to use the python's specific XML file handler


