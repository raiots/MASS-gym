import numpy as np


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



