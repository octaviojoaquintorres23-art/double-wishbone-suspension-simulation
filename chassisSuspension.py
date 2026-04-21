import math, numpy as np, copy, OKS_math, doubleAArmFS as DAAFS, trailingArmSuspension as TAS

class chassisSuspension():
    def __init__(self, SS_doubleAArmFS, SS_trailingArmSuspension):
        self.LH_FS = copy.deepcopy(SS_doubleAArmFS)
        self.LH_RS = copy.deepcopy(SS_trailingArmSuspension)
        self.RH_FS = copy.deepcopy(SS_doubleAArmFS)
        self.RH_RS = copy.deepcopy(SS_trailingArmSuspension)

        mirror = np.array([-1.0,1.0,1.0])

        self.RH_FS.A1 = self.RH_FS.A1 * mirror
        self.RH_FS.A2 = self.RH_FS.A2 * mirror
        self.RH_FS.B1 = self.RH_FS.B1 * mirror
        self.RH_FS.B2 = self.RH_FS.B2 * mirror
 
        self.RH_FS.TR1 = self.RH_FS.TR1 * mirror
        self.RH_FS.P1L = self.RH_FS.P1L * mirror
        self.RH_FS.P2L = self.RH_FS.P2L * mirror
        self.RH_FS.P3L = self.RH_FS.P3L * mirror
        self.RH_FS.DS1 = self.RH_FS.DS1 * mirror
        self.RH_FS.WAIL = self.RH_FS.WAIL * mirror
        self.RH_FS.WAL = self.RH_FS.WAL * mirror
        self.RH_FS.SENSE = self.RH_FS.SENSE * -1

        self.RH_RS.A1 = self.RH_RS.A1 * mirror
        self.RH_RS.B1 = self.RH_RS.B1 * mirror
        self.RH_RS.AB2 = self.RH_RS.AB2 * mirror
        self.RH_RS.AB2L = self.RH_RS.AB2L * mirror
        self.RH_RS.R1L = self.RH_RS.R1L * mirror
        self.RH_RS.R2L = self.RH_RS.R2L * mirror
        self.RH_RS.P1L = self.RH_RS.P1L * mirror
        self.RH_RS.P2L = self.RH_RS.P2L * mirror
        self.RH_RS.WAL = self.RH_RS.WAL * mirror
        self.RH_RS.DS1 = self.RH_RS.DS1 * mirror

        self.RH_RS.repossess()
        self.RH_FS.repossess()

        self.LH_FS_contactPatchC = np.array([0,0,0]) * 1.0
        self.RH_FS_contactPatchC = np.array([0,0,0]) * 1.0
        self.FS_rollCenterP = np.array([0,0,0]) * 1.0
        self.FS_rollCenterN = np.array([0,0,0]) * 1.0

        self.ackermanCenter = np.array([0,0,0]) * 1.0

        self.rebuild()
        
    def rebuild(self):
        self.LH_FS.rebuild()
        self.LH_RS.rebuild()
        self.RH_FS.rebuild()
        self.RH_RS.rebuild()

        deltaWA = self.LH_FS.WA - self.LH_FS.WAI
        horizontal = np.cross(deltaWA, np.array([0,1,0]))
        down = np.cross(deltaWA, horizontal)
        down = down / np.linalg.norm(down)
        self.LH_FS_contactPatchC = self.LH_FS.WA + 11*down

        deltaWA = self.RH_FS.WA - self.RH_FS.WAI
        horizontal = np.cross(deltaWA, np.array([0,1,0]))
        down = np.cross(deltaWA, horizontal)
        down = down / np.linalg.norm(down)
        self.RH_FS_contactPatchC = self.RH_FS.WA + 11*down

        RH_N = np.cross(self.RH_FS_contactPatchC - self.RH_FS.instanceAxisP, self.RH_FS.instanceAxisN)
        RH_N = RH_N / np.linalg.norm(RH_N)

        LH_N = np.cross(self.LH_FS_contactPatchC - self.LH_FS.instanceAxisP, self.LH_FS.instanceAxisN)
        LH_N = LH_N / np.linalg.norm(LH_N)

        P3n3 = OKS_math.intersectPlanePlane(self.RH_FS.instanceAxisP, RH_N, self.LH_FS.instanceAxisP, LH_N)
        self.FS_rollCenterP = P3n3[0]
        self.FS_rollCenterN = P3n3[1]

        deltaWA = self.LH_FS.WA - self.LH_FS.WAI
        LH_N = np.cross(deltaWA, np.array([0,1,0]))
        LH_N = LH_N / np.linalg.norm(LH_N)

        deltaWA = self.RH_FS.WA - self.RH_FS.WAI
        RH_N = np.cross(deltaWA, np.array([0,1,0]))
        RH_N = RH_N / np.linalg.norm(RH_N)

        P3n3 = OKS_math.intersectPlanePlane(self.LH_FS_contactPatchC, LH_N, self.RH_FS_contactPatchC, RH_N)
        self.ackermanCenter = P3n3[0]
