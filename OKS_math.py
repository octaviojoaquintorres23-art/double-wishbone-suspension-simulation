import numpy as np, math

#Makes a circle in 3D space, of center C2 and radius R2, planar to the plane specified
def intersectSpherePlane(C1, R1, P, n):
    success = True
    
    if (abs(np.dot(C1-P, n)) <= R1):
        R2 = math.sqrt(R1*R1 - np.dot(C1-P, n)*np.dot(C1-P, n))
    else:
        R2 = 0
        success = False
    
    C2 = C1 - n*np.dot(C1-P, n)
    return (C2, R2, success)

#Makes a circle in 3D space, of center C3 and radius R3, perpendicular to deltaC
def intersectSphereSphere(C1, R1, C2, R2):
    success = True
    
    deltaC = C2 - C1
    x1 = np.linalg.norm(deltaC)
    if (abs(R1 - R2) <= x1 and x1 <= R1 + R2):
        P = (R1*R1 - R2*R2 + x1*x1) / (2*x1*x1)
    else:
        P = 0
        success = False

    if (R1*R1-P*P*x1*x1 >= 0):
        R3 = math.sqrt(R1*R1-P*P*x1*x1)
    else:
        R3 = 0
        success = False
    
    C3 = C1 + P*deltaC
    
    return (C3, R3, success)

def solveBallHingeConstraint(C1,R1, C2, R2, n):
    C3R3 = intersectSpherePlane(C1, R1, C2, n)
    C3 = C3R3[0]
    R3 = C3R3[1]
    C4R4 = intersectSphereSphere(C3,R3,C2,R2)
    C4 = C4R4[0]
    R4 = C4R4[1]

    Gn = np.cross(C2 - C3,n)
    Gn = Gn / np.linalg.norm(Gn)

    return C4 + R4*Gn

def locatePoint(point, ref1L, ref2L, ref3L, ref1G, ref2G, ref3G):
    C1R1 = intersectSphereSphere(ref1G, np.linalg.norm(ref1L - point), ref2G, np.linalg.norm(ref2L - point))
    C1 = C1R1[0]
    R1 = C1R1[1]

    Pn = ref1G - ref2G
    Pn = Pn / np.linalg.norm(Pn)
    positivity = np.dot(ref3L - point, np.cross(ref3L - ref2L, ref1L - ref2L))
    if (positivity > 0):
        finalPoint = solveBallHingeConstraint(ref3G, np.linalg.norm(ref3L - point), C1, R1, Pn)
    else:
        finalPoint = solveBallHingeConstraint(ref3G, np.linalg.norm(ref3L - point), C1, R1, Pn*-1)
    return finalPoint

def vectorRotate(Vector, Axis, Theta):
    Rx = Axis[0]
    Ry = Axis[1]
    Rz = Axis[2]
    cos = math.cos(Theta)
    sin = math.sin(Theta)
    tempVector = np.array([Vector[0],Vector[1],Vector[2],1])
    transformationMatrix = np.array([
        [cos + Rx*Rx*(1-cos),Rx*Ry*(1-cos)-Rz*sin,Rx*Rz*(1-cos)+Ry*sin,0],
        [Ry*Rx*(1-cos)+Rz*sin, cos + Ry*Ry*(1-cos), Ry*Rz*(1-cos)-Rx*sin,0],
        [Rz*Rx*(1-cos)-Ry*sin, Rz*Ry*(1-cos)+Rx*sin, cos+Rz*Rz*(1-cos),0],
        [0,0,0,1]
         ])
    transformedVector = np.dot(transformationMatrix,tempVector)
    return np.array([transformedVector[0],transformedVector[1],transformedVector[2]])

def generateRotationMatrix(Axis, Theta):
    Rx = Axis[0]
    Ry = Axis[1]
    Rz = Axis[2]
    cos = math.cos(Theta)
    sin = math.sin(Theta)
    return np.array([
        [cos + Rx*Rx*(1-cos),Rx*Ry*(1-cos)-Rz*sin,Rx*Rz*(1-cos)+Ry*sin,0],
        [Ry*Rx*(1-cos)+Rz*sin, cos + Ry*Ry*(1-cos), Ry*Rz*(1-cos)-Rx*sin,0],
        [Rz*Rx*(1-cos)-Ry*sin, Rz*Ry*(1-cos)+Rx*sin, cos+Rz*Rz*(1-cos),0],
        [0,0,0,1]
         ])

#Returns length of ray
def raycast(P1, u, P2, n):
    return np.dot(n, P1-P2) / np.dot(u,-1*n)

#Makes a line in 3D space, of origin P3 and direction n3, planar with both planes specified
def intersectPlanePlane(P1, n1, P2, n2):
    n3 = np.cross(n1,n2)
    n3 = n3 / np.linalg.norm(n3)

    u = np.cross(n1,n3)
    u = u / np.linalg.norm(u)
    P3 = P1 + u*raycast(P1, u, P2, n2)
    return (P3, n3)
