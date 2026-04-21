import pygame, math, numpy as np, random, chassisSuspension , doubleAArmFS as DAAFS, trailingArmSuspension as TAS
pygame.init()

SCREEN_WIDTH = 1600
SCREEN_HEIGHT = 900
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()

# Standard units are: IMPERIAL
# Length: Inch
# Mass: Pound Mass
# Time: Second

#Pixels per inch
GRAPHICS_SCALE = 8

def drawThirdAngleProjection(position, offset, Points, Lines):
    global colorCatalog
    global screen
    global SCREEN_HEIGHT
    global GRAPHICS_SCALE
    
    #XY view
    for i in range(len(Lines)):
        item = Lines[i]
        pygame.draw.line(screen, colorCatalog[i % len(colorCatalog)],
                         (position[0] + item[0][0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[0][1]*GRAPHICS_SCALE)),
                         (position[0] + item[1][0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[1][1]*GRAPHICS_SCALE)),1)

    for i in range(len(Points)):
        item = Points[i]
        pygame.draw.circle(screen, colorCatalog[i % len(colorCatalog)], (position[0] + item[0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[1]*GRAPHICS_SCALE)), 2)

    #XZ view
    for i in range(len(Lines)):
        item = Lines[i]
        pygame.draw.line(screen, colorCatalog[i % len(colorCatalog)],
                         (position[0] + item[0][0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (offset[1]*GRAPHICS_SCALE + position[1] - item[0][2]*GRAPHICS_SCALE)),
                         (position[0] + item[1][0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (offset[1]*GRAPHICS_SCALE + position[1] - item[1][2]*GRAPHICS_SCALE)),1)

    for i in range(len(Points)):
        item = Points[i]
        pygame.draw.circle(screen, colorCatalog[i % len(colorCatalog)], (position[0] + item[0]*GRAPHICS_SCALE, SCREEN_HEIGHT - (offset[1]*GRAPHICS_SCALE + position[1] - item[2]*GRAPHICS_SCALE)), 2)

    #ZY view
    for i in range(len(Lines)):
        item = Lines[i]
        pygame.draw.line(screen, colorCatalog[i % len(colorCatalog)],
                         (offset[0]*GRAPHICS_SCALE + position[0] - item[0][2]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[0][1]*GRAPHICS_SCALE)),
                         (offset[0]*GRAPHICS_SCALE + position[0] - item[1][2]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[1][1]*GRAPHICS_SCALE)),1)

    for i in range(len(Points)):
        item = Points[i]
        pygame.draw.circle(screen, colorCatalog[i % len(colorCatalog)], (offset[0]*GRAPHICS_SCALE + position[0] - item[2]*GRAPHICS_SCALE, SCREEN_HEIGHT - (position[1] + item[1]*GRAPHICS_SCALE)), 2)

def writeStudyToFile(filename, studyData):
        file = open("program_output/" + filename + ".txt","w")
        file.write("")
        file.close()
        file = open("program_output/" + filename + ".txt","a")
        for i in studyData:
            file.write(str(i) + "\n")
        file.close()

A1 = np.array([5.848,2.72,-29.428])
B1 = np.array([5.857,6.231,-29.428])
AB2 = np.array([10,0,-1.969])
UR = 15.06
LR = 15.056
AB2L = np.array([10,0,-1.969])
R1L = np.array([17.822,-3.976,-25.602])
R2L = np.array([17.822,-1.473,-25.602])
P1L = np.array([18.332,-5.717,-29.235])
P2L = np.array([18.323,-2.217,-29.234])
WAL = np.array([19.132,-3.972,-25.691])
DS1 = np.array([5.722,3.867,-25.106])
DS2 = 1.75

chassisBottomHeight = -0.625
wheelRadius = 11

trailingArmClass1 = TAS.trailingArmSuspension(A1,B1,AB2,UR,LR,AB2L,R1L,R2L,P1L,P2L,WAL,DS1,DS2, 1)

A1 = np.array([5.82499986, .36466116, 24.53223804])
A2 = np.array([5.82499986, 2.41370966, 36.15296953])
B1 = np.array([7.26651297, 5.87506307, 24.82988862])
B2 = np.array([7.26651297, 7.48999112, 33.98860072])
C1 = 11.8 / 2.0
C2 = 20.16501225
D1 = 9.30000000 / 2.0
D2 = 17.16595115
E = 7.08725583
tieRodBodyLink = np.array([8.35000000, 4.82010367, 34.89948930])
tieRodLength = 17.85000000

P1L = np.array([25.36891890,-3.50154795,31.20497126])
P2L = np.array([24.09867966,3.36501933,29.99421080])
P3L = np.array([25.63175150,.69016969,33.19443492])
DS1 = np.array([7.099,4.05883242,29.97223454])
WAIL = np.array([23.59613567,-.08294757,30.63385048])
WAL = np.array([27.79353353,-.00598987,30.54553345])

doubleAArmClass1 = DAAFS.doubleAArmFS(A1, A2, B1, B2, C1, C2, D1, D2, tieRodBodyLink, tieRodLength, P1L, P2L, P3L, DS1, WAIL, WAL, 1)

chassis1 = chassisSuspension.chassisSuspension(doubleAArmClass1, trailingArmClass1)

chassis1.LH_FS.TR1[0] += .5
chassis1.RH_FS.TR1[0] += .5

studyCAMBER_R = []
studyCASTER_R = []
studyTOE_R = []
studySHAFT_LENGTH_R = []
studyTHETA_R = []
studyRIDE_HEIGHT_R = []
RUNNING_STUDY_R = False

studyCAMBER_F = []
studyCASTER_F = []
studyTOE_F = []
studySHAFT_LENGTH_F = []
studyTHETA_F = []
studyRIDE_HEIGHT_F = []
RUNNING_STUDY_F = False

colorCatalog = [
    (255,0,0),
    (255,125,0),
    (125,255,0),
    (255,255,0),
    (0,255,0),
    (0,255,125),
    (0,125,255),
    (0,255,255),
    (0,0,255),
    (125,0,255),
    (255,0,125),
    (255,0,255)
    ]

cameraX = 400
cameraY = 0

running = True

while running:
    clock.tick(60)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((0,0,0))

    keys = pygame.key.get_pressed()

    if keys[pygame.K_RIGHT]:
        cameraX -= 10
    if keys[pygame.K_LEFT]:
        cameraX += 10
    if keys[pygame.K_UP]:
        cameraY -= 10
    if keys[pygame.K_DOWN]:
        cameraY += 10
    if keys[pygame.K_k]:
        GRAPHICS_SCALE = GRAPHICS_SCALE / 1.01
        cameraX = (cameraX - SCREEN_WIDTH*0.5) / 1.01 + SCREEN_WIDTH*0.5
        cameraY = (cameraY - SCREEN_HEIGHT*0.5) / 1.01 + SCREEN_HEIGHT*0.5
    if keys[pygame.K_i]:
        GRAPHICS_SCALE = GRAPHICS_SCALE * 1.01
        cameraX = (cameraX - SCREEN_WIDTH*0.5) * 1.01 + SCREEN_WIDTH*0.5
        cameraY = (cameraY - SCREEN_HEIGHT*0.5) * 1.01 + SCREEN_HEIGHT*0.5
    if keys[pygame.K_a]:
        chassis1.LH_FS.TR1[0] += .05
        chassis1.RH_FS.TR1[0] += .05
    if keys[pygame.K_d]:
        chassis1.LH_FS.TR1[0] -= .05
        chassis1.RH_FS.TR1[0] -= .05

##    if keys[pygame.K_SPACE] and not RUNNING_STUDY_R and not RUNNING_STUDY_F:
##        studyCAMBER_R.clear()
##        studyCASTER_R.clear()
##        studyTOE_R.clear()
##        studySHAFT_LENGTH_R.clear()
##        studyTHETA_R.clear()
##        studyRIDE_HEIGHT_R.clear()
##        RUNNING_STUDY_R = True
##        thetaR = -30.0*math.pi/180
##
##        studyCAMBER_F.clear()
##        studyCASTER_F.clear()
##        studyTOE_F.clear()
##        studySHAFT_LENGTH_F.clear()
##        studyTHETA_F.clear()
##        studyRIDE_HEIGHT_F.clear()
##        RUNNING_STUDY_F = True
##        thetaF = -30.0*math.pi/180

    if not RUNNING_STUDY_R:
        thetaR = (30 + 15*math.sin(pygame.time.get_ticks()/1000))*math.pi/180
    else:
        thetaR += .1*math.pi/180

    if not RUNNING_STUDY_F:
        thetaF = (15 + 15*math.sin(pygame.time.get_ticks()/1000))*math.pi/180
    else:
        thetaF += .1*math.pi/180

    if thetaR >= 45.0*math.pi/180:
        RUNNING_STUDY_R = False

        writeStudyToFile("RS_CAMBER", studyCAMBER_R)
        writeStudyToFile("RS_CASTER", studyCASTER_R)
        writeStudyToFile("RS_TOE", studyTOE_R)
        writeStudyToFile("RS_SHAFT_LENGTH", studySHAFT_LENGTH_R)
        writeStudyToFile("RS_THETA", studyTHETA_R)
        writeStudyToFile("RS_RIDE_HEIGHT", studyRIDE_HEIGHT_R)

    if thetaF >= 45.0*math.pi/180:
        RUNNING_STUDY_F = False

        writeStudyToFile("FS_CAMBER", studyCAMBER_F)
        writeStudyToFile("FS_CASTER", studyCASTER_F)
        writeStudyToFile("FS_TOE", studyTOE_F)
        writeStudyToFile("FS_SHAFT_LENGTH", studySHAFT_LENGTH_F)
        writeStudyToFile("FS_THETA", studyTHETA_F)
        writeStudyToFile("FS_RIDE_HEIGHT", studyRIDE_HEIGHT_F)
        
    phi =  (15 + 30*math.sin(pygame.time.get_ticks() / 60.0))*math.pi/180#(10 + 30*math.sin(50 * math.sin(pygame.time.get_ticks() / 3000.0)))*math.pi/180

    trailingArmClass1.theta = thetaR
    trailingArmClass1.phi = phi
    trailingArmClass1.rebuild()

    chassis1.LH_FS.WHEEL_THETA = (5*pygame.time.get_ticks()/60.0 + 8*6*math.sin(pygame.time.get_ticks()/360.0))*math.pi/180
    chassis1.RH_FS.WHEEL_THETA = (5*pygame.time.get_ticks()/60.0 + 8*6*math.sin(pygame.time.get_ticks()/360.0))*math.pi/180

    chassis1.LH_FS.theta = thetaF
    chassis1.RH_FS.theta = thetaF#(15 + 15*math.sin(pygame.time.get_ticks()/300))*math.pi/180
    chassis1.rebuild()

    if RUNNING_STUDY_R:
        normal = trailingArmClass1.WA - trailingArmClass1.WAC
        normal = normal / np.linalg.norm(normal)
        CAMBER = math.acos(np.dot(np.array([0,1,0]),normal))*180/math.pi - 90

        swivelAxis = (trailingArmClass1.R2 - trailingArmClass1.R1)
        swivelAxis[0] = 0
        swivelAxis = swivelAxis / np.linalg.norm(swivelAxis)
        CASTER = math.acos(np.dot(np.array([0,0,1]),swivelAxis))*180/math.pi - 90

        normal = trailingArmClass1.WA - trailingArmClass1.WAC
        normal[1] = 0
        normal = normal / np.linalg.norm(normal)
        TOE = math.acos(np.dot(np.array([0,0,-1]),normal))*180/math.pi - 90

        SHAFT_LENGTH = np.linalg.norm(trailingArmClass1.DS1 - trailingArmClass1.WAI)
        RIDE_HEIGHT = wheelRadius + chassisBottomHeight - trailingArmClass1.P1[1]

        studyCAMBER_R.append(CAMBER)
        studyCASTER_R.append(CASTER)
        studyTOE_R.append(TOE)
        studySHAFT_LENGTH_R.append(SHAFT_LENGTH)
        studyTHETA_R.append(thetaR)
        studyRIDE_HEIGHT_R.append(RIDE_HEIGHT)
        
    if RUNNING_STUDY_F:
        normal = uprightWA - uprightWAI
        normal = normal / np.linalg.norm(normal)
        CAMBER = math.acos(np.dot(np.array([0,1,0]),normal))*180/math.pi - 90

        swivelAxis = (doubleAArmClass1.P2 - doubleAArmClass1.P1)
        swivelAxis[0] = 0
        swivelAxis = swivelAxis / np.linalg.norm(swivelAxis)
        CASTER = math.acos(np.dot(np.array([0,0,1]),swivelAxis))*180/math.pi - 90

        normal = uprightWA - uprightWAI
        normal[1] = 0
        normal = normal / np.linalg.norm(normal)
        TOE = math.acos(np.dot(np.array([0,0,-1]),normal))*180/math.pi - 90

        SHAFT_LENGTH = np.linalg.norm(FS_DS1 - uprightWAI)
        RIDE_HEIGHT = wheelRadius + chassisBottomHeight - doubleAArmClass1.P1[1]

        studyCAMBER_F.append(CAMBER)
        studyCASTER_F.append(CASTER)
        studyTOE_F.append(TOE)
        studySHAFT_LENGTH_F.append(SHAFT_LENGTH)
        studyTHETA_F.append(thetaF)
        studyRIDE_HEIGHT_F.append(RIDE_HEIGHT)

    globalLineSegmentsToDraw = [
        (trailingArmClass1.AB2,trailingArmClass1.R2),
        (trailingArmClass1.AB2,trailingArmClass1.R1),
        (trailingArmClass1.R1,trailingArmClass1.R2),
        (trailingArmClass1.R1,trailingArmClass1.P1),
        (trailingArmClass1.R1,trailingArmClass1.P2),
        (trailingArmClass1.R2,trailingArmClass1.P1),
        (trailingArmClass1.R2,trailingArmClass1.P2),
        (trailingArmClass1.P1,trailingArmClass1.P2),
        (trailingArmClass1.P1,trailingArmClass1.A1),
        (trailingArmClass1.P2,trailingArmClass1.B1),
        (trailingArmClass1.WA,trailingArmClass1.WAC),
        (trailingArmClass1.DS1,trailingArmClass1.WAI),
        (trailingArmClass1.WAI,trailingArmClass1.WAC)
        ]
    globalPointsToDraw = [
        trailingArmClass1.AB2,
        trailingArmClass1.R2,
        trailingArmClass1.R1,
        trailingArmClass1.P2,
        trailingArmClass1.P1,
        trailingArmClass1.B1,
        trailingArmClass1.A1,
        trailingArmClass1.WA,
        trailingArmClass1.WAC,
        trailingArmClass1.WAI,
        trailingArmClass1.DS1,
        np.array([0,0,0]),
        np.array([0,chassisBottomHeight,0]),
        trailingArmClass1.P1 + np.array([0,-1*wheelRadius,0])
        ]
    localLineSegmentsToDraw = [
        (trailingArmClass1.AB2L,trailingArmClass1.R2L),
        (trailingArmClass1.AB2L,trailingArmClass1.R1L),
        (trailingArmClass1.R1L,trailingArmClass1.R2L),
        (trailingArmClass1.R1L,trailingArmClass1.P1L),
        (trailingArmClass1.R1L,trailingArmClass1.P2L),
        (trailingArmClass1.R2L,trailingArmClass1.P1L),
        (trailingArmClass1.R2L,trailingArmClass1.P2L),
        (trailingArmClass1.P2L,trailingArmClass1.P1L),
        (trailingArmClass1.WAL,trailingArmClass1.WACL),
        (trailingArmClass1.WAILT,trailingArmClass1.WACL)
        ]
    localPointsToDraw = [
        trailingArmClass1.AB2L,
        trailingArmClass1.R1L,
        trailingArmClass1.R2L,
        trailingArmClass1.P1L,
        trailingArmClass1.P2L,
        trailingArmClass1.WAL,
        trailingArmClass1.WALT,
        trailingArmClass1.WACL,
        trailingArmClass1.WAILT
        ]

    for i in range(10):
        pygame.draw.line(screen, (100,100,100), (cameraX + i*10*GRAPHICS_SCALE,0),(cameraX + i*10*GRAPHICS_SCALE,SCREEN_HEIGHT))
    for i in range(10):
        pygame.draw.line(screen, (100,100,100), (0,SCREEN_HEIGHT - (cameraY + i*10*GRAPHICS_SCALE)),(SCREEN_WIDTH,SCREEN_HEIGHT - (cameraY + i*10*GRAPHICS_SCALE)))

    # GLOBAL VIEW

    drawThirdAngleProjection((210*GRAPHICS_SCALE + cameraX, 20*GRAPHICS_SCALE + cameraY),(40,40),globalPointsToDraw,globalLineSegmentsToDraw)

    # LOCAL VIEWS

    drawThirdAngleProjection((280*GRAPHICS_SCALE + cameraX, 20*GRAPHICS_SCALE + cameraY),(40,40),localPointsToDraw, localLineSegmentsToDraw)

    globalLineSegmentsToDraw = [
        (chassis1.LH_FS.A1, chassis1.LH_FS.A2),
        (chassis1.LH_FS.B1, chassis1.LH_FS.B2),
        (chassis1.LH_FS.equivalentAArm.hingeCenterA, chassis1.LH_FS.P1),
        (chassis1.LH_FS.equivalentAArm.hingeCenterB, chassis1.LH_FS.P2),
        (chassis1.LH_FS.P1, chassis1.LH_FS.P2),
        (chassis1.LH_FS.TR1, chassis1.LH_FS.P3),
        (chassis1.LH_FS.WAI, chassis1.LH_FS.WA),
        (chassis1.LH_FS.DS1, chassis1.LH_FS.WAI),
        (chassis1.LH_FS.WA, chassis1.LH_FS.WA + (chassis1.LH_FS.WA - chassis1.LH_FS.WAI)*-10),
        
        (chassis1.RH_FS.A1, chassis1.RH_FS.A2),
        (chassis1.RH_FS.B1, chassis1.RH_FS.B2),
        (chassis1.RH_FS.equivalentAArm.hingeCenterA, chassis1.RH_FS.P1),
        (chassis1.RH_FS.equivalentAArm.hingeCenterB, chassis1.RH_FS.P2),
        (chassis1.RH_FS.P1, chassis1.RH_FS.P2),
        (chassis1.RH_FS.TR1, chassis1.RH_FS.P3),
        (chassis1.RH_FS.WAI, chassis1.RH_FS.WA),
        (chassis1.RH_FS.DS1, chassis1.RH_FS.WAI),
        (chassis1.RH_FS.WA, chassis1.RH_FS.WA + (chassis1.RH_FS.WA - chassis1.RH_FS.WAI)*10),

        (chassis1.FS_rollCenterP, chassis1.FS_rollCenterP + 10*chassis1.FS_rollCenterN)
        ]
    globalPointsToDraw = [
        np.array([0,0,0]),
        np.array([0,chassisBottomHeight,0]),
        chassis1.LH_FS.A1,
        chassis1.LH_FS.A2,
        chassis1.LH_FS.B1,
        chassis1.LH_FS.B2,
        chassis1.LH_FS.P1,
        chassis1.LH_FS.P2,
        chassis1.LH_FS.P3,
        chassis1.LH_FS.TR1,
        chassis1.LH_FS.WAI,
        chassis1.LH_FS.WA,
        chassis1.LH_FS.DS1,
        #chassis1.LH_FS.instanceAxisP,
        chassis1.LH_FS_contactPatchC,

        chassis1.RH_FS.A1,
        chassis1.RH_FS.A2,
        chassis1.RH_FS.B1,
        chassis1.RH_FS.B2,
        chassis1.RH_FS.P1,
        chassis1.RH_FS.P2,
        chassis1.RH_FS.P3,
        chassis1.RH_FS.TR1,
        chassis1.RH_FS.WAI,
        chassis1.RH_FS.WA,
        chassis1.RH_FS.DS1,
        #chassis1.RH_FS.instanceAxisP,
        chassis1.RH_FS_contactPatchC,

        chassis1.FS_rollCenterP,
        chassis1.ackermanCenter
        ]
    globalPointsToDraw = globalPointsToDraw + chassis1.LH_FS.WHEEL
    globalPointsToDraw = globalPointsToDraw + chassis1.RH_FS.WHEEL

    # GLOBAL VIEW

    drawThirdAngleProjection((50*GRAPHICS_SCALE + cameraX, 20*GRAPHICS_SCALE + cameraY),(100,100),globalPointsToDraw,globalLineSegmentsToDraw)
        
    pygame.display.flip()
pygame.quit()
