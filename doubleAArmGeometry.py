import numpy as np, OKS_math

class doubleAArmGeometry():
    def __init__(self, A1, A2, B1, B2, C1, C2, D1, D2, E, SENSE):
        self.A1 = A1 * 1.0
        self.A2 = A2 * 1.0
        self.B1 = B1 * 1.0
        self.B2 = B2 * 1.0
        self.C1 = C1 * 1.0
        self.C2 = C2 * 1.0
        self.D1 = D1 * 1.0
        self.D2 = D2 * 1.0
        self.E = E * 1.0
        self.SENSE = SENSE * 1.0
        self.theta = 0.0
        self.C2n = np.array([1,0,0]) * 1.0
        self.P1 = np.array([0,0,0]) * 1.0
        self.P2 = np.array([0,0,0]) * 1.0
        self.hingeCenterA = np.array([0,0,0]) * 1.0
        self.hingeCenterB = np.array([0,0,0]) * 1.0
        self.instanceAxisP = np.array([0,0,0]) * 1.0
        self.instanceAxisN = np.array([0,0,0]) * 1.0
        self.rebuild()

    def rebuild(self):
        A = self.A1 - self.A2
        An = A / np.linalg.norm(A)

        self.C2n = np.cross(A,np.array([0,1,0]))
        self.C2n = self.C2n / np.linalg.norm(self.C2n)
        self.C2n = OKS_math.vectorRotate(self.C2n, An*self.SENSE, self.theta)
        self.C2n = self.C2n * self.SENSE

        self.P1 = self.A1 - self.C1*An + self.C2*self.C2n

        B = self.B1 - self.B2
        Bn = B / np.linalg.norm(B)
        self.hingeCenterB = self.B1 - self.D1*Bn
        self.hingeCenterA = self.A1 - self.C1*An

        self.P2 = OKS_math.solveBallHingeConstraint(self.P1, self.E, self.hingeCenterB, self.D2, -1 * Bn * self.SENSE)

        V1n = np.cross(self.C2n, An)
        V1n = V1n / np.linalg.norm(V1n)

        V2n = np.cross(self.P2 - self.hingeCenterB, Bn)
        V2n = V2n / np.linalg.norm(V2n)

        P3n3 = OKS_math.intersectPlanePlane(self.P1,V1n,self.P2,V2n)
        self.instanceAxisP = P3n3[0]
        self.instanceAxisN = P3n3[1]
