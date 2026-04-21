import math, numpy as np, OKS_math, doubleAArmGeometry as DAAG

class doubleAArmFS():
    def __init__(self, A1, A2, B1, B2, C1, C2, D1, D2, TR1, TR2, P1L, P2L, P3L, DS1, WAIL, WAL, SENSE):
        self.A1 = A1 * 1.0
        self.A2 = A2 * 1.0
        self.B1 = B1 * 1.0
        self.B2 = B2 * 1.0
        self.C1 = C1 * 1.0
        self.C2 = C2 * 1.0
        self.D1 = D1 * 1.0
        self.D2 = D2 * 1.0
        self.TR1 = TR1 * 1.0
        self.TR2 = TR2 * 1.0
        self.P1L = P1L * 1.0
        self.P2L = P2L * 1.0
        self.E = np.linalg.norm(self.P1L - self.P2L)
        self.P3L = P3L * 1.0
        self.DS1 = DS1 * 1.0
        self.WAIL = WAIL * 1.0
        self.WAL = WAL * 1.0
        self.theta = 0.0
        self.P1 = np.array([0,0,0]) * 1.0
        self.P2 = np.array([0,0,0]) * 1.0
        self.P3 = np.array([0,0,0]) * 1.0
        self.WAI = np.array([0,0,0]) * 1.0
        self.WA = np.array([0,0,0]) * 1.0
        self.instanceAxisP = np.array([0,0,0]) * 1.0
        self.instanceAxisN = np.array([0,0,0]) * 1.0

        self.WHEEL_MODEL = []
        self.WHEEL_THETA = 0
        self.WHEEL = []

        for i in range(30):
            angle = i * 2*math.pi / 30.0
            self.WHEEL_MODEL.append(np.array([-3,11*math.sin(angle),11*math.cos(angle)]))
            self.WHEEL_MODEL.append(np.array([3,11*math.sin(angle),11*math.cos(angle)]))

        self.SENSE = SENSE

        self.equivalentAArm = DAAG.doubleAArmGeometry(self.A1, self.A2, self.B1, self.B2, self.C1, self.C2, self.D1, self.D2, self.E, SENSE)

        self.rebuild()
    def rebuild(self):

        self.equivalentAArm.theta = self.theta
        self.equivalentAArm.rebuild()

        self.P1 = self.equivalentAArm.P1
        self.P2 = self.equivalentAArm.P2
        self.instanceAxisP = self.equivalentAArm.instanceAxisP
        self.instanceAxisN = self.equivalentAArm.instanceAxisN

        nL = self.P1L - self.P2L
        nL = nL / np.linalg.norm(nL)
        tieRodHeight = np.dot(self.P3L - self.P1L, nL)
        tieRodArm = np.linalg.norm((self.P3L - self.P1L) - nL*tieRodHeight)
        tieRodHeight = abs(tieRodHeight)
        tieRodArm = abs(tieRodArm)

        n = self.P1 - self.P2
        n = n / np.linalg.norm(n)

        self.P3 = OKS_math.solveBallHingeConstraint(self.TR1, self.TR2, self.P1 - n*tieRodHeight, tieRodArm, -1*n*self.SENSE)
        self.WAI = OKS_math.locatePoint(self.WAIL, self.P1L, self.P2L, self.P3L, self.P1, self.P2, self.P3)
        self.WA = OKS_math.locatePoint(self.WAL, self.P1L, self.P2L, self.P3L, self.P1, self.P2, self.P3)

        self.WHEEL = self.WHEEL_MODEL[:]
        n = self.WA - self.WAI
        n[1] = 0
        n = n*self.SENSE / np.linalg.norm(n)
        rotate1 = OKS_math.generateRotationMatrix(np.array([0,1,0]),math.acos(np.dot(n,np.array([0,0,1]))) - 90*math.pi/180.0)

        n = self.WA - self.WAI
        n[2] = 0
        n = n*self.SENSE / np.linalg.norm(n)
        rotate2 = OKS_math.generateRotationMatrix(np.array([0,0,1]),math.acos(np.dot(n,np.array([0,-1,0]))) - 90*math.pi/180.0)

        rotate3 = OKS_math.generateRotationMatrix(np.array([1,0,0]),self.WHEEL_THETA)
        for i in range(len(self.WHEEL)):

            tempVector = np.array([self.WHEEL[i][0],self.WHEEL[i][1],self.WHEEL[i][2],1])
            tempVector = np.dot(rotate3, tempVector)
            tempVector = np.dot(rotate1, tempVector)
            tempVector = np.dot(rotate2, tempVector)
            self.WHEEL[i] = np.array([tempVector[0],tempVector[1],tempVector[2]])
            self.WHEEL[i] = self.WHEEL[i] + self.WA

    # Pointers moment
    def repossess(self):

        self.equivalentAArm.A1 = self.A1
        self.equivalentAArm.A2 = self.A2
        self.equivalentAArm.B1 = self.B1
        self.equivalentAArm.B2 = self.B2
        self.equivalentAArm.C1 = self.C1
        self.equivalentAArm.C2 = self.C2
        self.equivalentAArm.D1 = self.D1
        self.equivalentAArm.D2 = self.D2
        self.equivalentAArm.E = self.E
        self.equivalentAArm.SENSE = self.SENSE
        
