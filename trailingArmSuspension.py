import math, numpy as np, OKS_math, doubleAArmGeometry as DAAG

class trailingArmSuspension():
    def __init__(self, A1, B1, AB2, UR, LR, AB2L, R1L, R2L, P1L, P2L, WAL, DS1, DS2, SENSE):
        self.A1 = A1 * 1.0
        self.B1 = B1 * 1.0
        self.AB2 = AB2 * 1.0
        self.UR = UR * 1.0
        self.LR = LR * 1.0
        self.AB2L = AB2L * 1.0
        self.R1L = R1L * 1.0
        self.R2L = R2L * 1.0
        self.P1L = P1L * 1.0
        self.P2L = P2L * 1.0
        self.WAL = WAL * 1.0
        self.WALT = WAL
        self.DS1 = DS1
        self.DS2 = DS2
        self.SENSE = SENSE
        self.theta = 0.0
        self.phi = 0.0

        self.R1 = np.array([0,0,0]) * 1.0
        self.R2 = np.array([0,0,0]) * 1.0
        self.P1 = np.array([0,0,0]) * 1.0
        self.P2 = np.array([0,0,0]) * 1.0
        self.WA = np.array([0,0,0]) * 1.0
        self.WACL = np.array([0,0,0]) * 1.0
        self.WAC = np.array([0,0,0]) * 1.0
        self.WAI = np.array([0,0,0]) * 1.0
        self.WAILT = np.array([0,0,0]) * 1.0

        self.C1 = 1.0
        self.C2 = 1.0
        self.D1 = 1.0
        self.D2 = 1.0
        self.E = 1.0

        self.equivalentAArm = DAAG.doubleAArmGeometry(self.A1, self.AB2, self.B1, self.AB2, self.C1, self.C2, self.D1, self.D2, self.E, SENSE)

        self.rebuild()
    def rebuild(self):
        R = self.R1L - self.R2L
        Rn = R / np.linalg.norm(R)

        self.WACL = self.R2L + np.dot(self.WAL - self.R2L, Rn) * Rn
        
        self.WALT = self.R1L + OKS_math.vectorRotate(self.WAL - self.R1L, Rn, self.phi)
        self.WAILT = self.WACL + (self.WACL - self.WALT) * self.DS2 / np.linalg.norm(self.WACL - self.WALT)

        # I believe that a much simpler way of calculating the equivalent arm lengths is to use projections
        # But if it aint broke, don't fix it
        LI = np.linalg.norm(self.P1L - self.AB2L)
        UI = np.linalg.norm(self.P2L - self.AB2L)

        SL = (np.linalg.norm(self.A1 - self.AB2) + self.LR + LI) * 0.5
        SU = (np.linalg.norm(self.B1 - self.AB2) + self.UR + UI) * 0.5
        self.C2 = 2 * math.sqrt(SL * (SL - np.linalg.norm(self.A1 - self.AB2)) * (SL - self.LR) * (SL - LI)) / np.linalg.norm(self.A1 - self.AB2)
        self.D2 = 2 * math.sqrt(SU * (SU - np.linalg.norm(self.B1 - self.AB2)) * (SU - self.UR) * (SU - UI)) / np.linalg.norm(self.B1 - self.AB2)
        self.C1 = math.sqrt(self.LR*self.LR - self.C2*self.C2)
        self.D1 = math.sqrt(self.UR*self.UR - self.D2*self.D2)
        self.E = np.linalg.norm(self.P1L - self.P2L)

        self.equivalentAArm.C1 = self.C1
        self.equivalentAArm.C2 = self.C2
        self.equivalentAArm.D1 = self.D1
        self.equivalentAArm.D2 = self.D2
        self.equivalentAArm.E = self.E
        self.equivalentAArm.theta = self.theta

        self.equivalentAArm.rebuild()

        self.P1 = self.equivalentAArm.P1
        self.P2 = self.equivalentAArm.P2

        self.R1 = OKS_math.locatePoint(self.R1L, self.P1L, self.P2L, self.AB2L, self.P1, self.P2, self.AB2)
        self.R2 = OKS_math.locatePoint(self.R2L, self.P1L, self.P2L, self.AB2L, self.P1, self.P2, self.AB2)
        self.WAC = OKS_math.locatePoint(self.WACL, self.P1L, self.P2L, self.AB2L, self.P1, self.P2, self.AB2)
        self.WA = OKS_math.locatePoint(self.WALT, self.P1L, self.P2L, self.AB2L, self.P1, self.P2, self.AB2)
        self.WAI = OKS_math.locatePoint(self.WAILT, self.P1L, self.P2L, self.AB2L, self.P1, self.P2, self.AB2)

    def repossess(self):

        self.equivalentAArm.A1 = self.A1
        self.equivalentAArm.A2 = self.AB2
        self.equivalentAArm.B1 = self.B1
        self.equivalentAArm.B2 = self.AB2
        self.equivalentAArm.C1 = self.C1
        self.equivalentAArm.C2 = self.C2
        self.equivalentAArm.D1 = self.D1
        self.equivalentAArm.D2 = self.D2
        self.equivalentAArm.E = self.E
        self.equivalentAArm.SENSE = self.SENSE
