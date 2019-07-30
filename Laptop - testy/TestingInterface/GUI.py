import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
import threading

HALF_SHIP_WIDTH = 5
HALF_SHIP_HEIGHT = 0.5
HALF_MOTOR_WIDTH = 1
MOTOR_HEIGHT = 2
MAX_MOTORS_POWER = 100

# DEFINING ALL VERTICES AND EDGES:
# I teat them as constant values that's why i decided not to define them as instance variables
coordinateSystemVertices=(
    (0, 0, 0),
    (20, 0, 0),  # X
    (0, 20, 0),  # Y in code, but Z in real
    (0, 0, 20),   # Z in code...
    #  arrows :
    (20 - 1, 1, 0),
    (20 - 1, -1, 0),
    (1, 20 - 1, 0),
    (-1, 20 - 1, 0),
    (1, 0, 20 - 1),
    (-1, 0, 20 - 1),
)

boardVertices=(
    (-HALF_SHIP_WIDTH, -HALF_SHIP_HEIGHT, -HALF_SHIP_WIDTH),  # 0
    (HALF_SHIP_WIDTH, -HALF_SHIP_HEIGHT, -HALF_SHIP_WIDTH),  # 1 ...
    (HALF_SHIP_WIDTH, -HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH),
    (-HALF_SHIP_WIDTH, -HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH),
    (-HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, -HALF_SHIP_WIDTH),
    (HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, -HALF_SHIP_WIDTH),
    (HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH),
    (-HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH)
)

motor1Vertices=(
    (boardVertices[4][0] - HALF_MOTOR_WIDTH, boardVertices[4][1], boardVertices[4][2] - HALF_MOTOR_WIDTH),
    (boardVertices[4][0] + HALF_MOTOR_WIDTH, boardVertices[4][1], boardVertices[4][2] - HALF_MOTOR_WIDTH),
    (boardVertices[4][0] + HALF_MOTOR_WIDTH, boardVertices[4][1], boardVertices[4][2] + HALF_MOTOR_WIDTH),
    (boardVertices[4][0] - HALF_MOTOR_WIDTH, boardVertices[4][1], boardVertices[4][2] + HALF_MOTOR_WIDTH),
    (boardVertices[4][0], boardVertices[4][1] + MOTOR_HEIGHT, boardVertices[4][2])
)

motor2Vertices=(
    (boardVertices[5][0] - HALF_MOTOR_WIDTH, boardVertices[5][1], boardVertices[5][2] - HALF_MOTOR_WIDTH),
    (boardVertices[5][0] + HALF_MOTOR_WIDTH, boardVertices[5][1], boardVertices[5][2] - HALF_MOTOR_WIDTH),
    (boardVertices[5][0] + HALF_MOTOR_WIDTH, boardVertices[5][1], boardVertices[5][2] + HALF_MOTOR_WIDTH),
    (boardVertices[5][0] - HALF_MOTOR_WIDTH, boardVertices[5][1], boardVertices[5][2] + HALF_MOTOR_WIDTH),
    (boardVertices[5][0], boardVertices[5][1] + MOTOR_HEIGHT, boardVertices[5][2])
)

motor3Vertices=(
    (-HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH),
    (HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH),
    (HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH),
    (-HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT, HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH),
    (0, HALF_SHIP_HEIGHT + MOTOR_HEIGHT, HALF_SHIP_WIDTH)
)

motor4Vertices=(
    (HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT + HALF_MOTOR_WIDTH, 0),
    (HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT + HALF_MOTOR_WIDTH, 0),
    (HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT - HALF_MOTOR_WIDTH, 0),
    (HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT - HALF_MOTOR_WIDTH, 0),
    (HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, -MOTOR_HEIGHT)
)

motor5Vertices=(
    (-HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT + HALF_MOTOR_WIDTH, 0),
    (-HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT + HALF_MOTOR_WIDTH, 0),
    (-HALF_SHIP_WIDTH + HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT - HALF_MOTOR_WIDTH, 0),
    (-HALF_SHIP_WIDTH - HALF_MOTOR_WIDTH, HALF_SHIP_HEIGHT - HALF_MOTOR_WIDTH, 0),
    (-HALF_SHIP_WIDTH, HALF_SHIP_HEIGHT, -MOTOR_HEIGHT)
)

motorsVertices = (motor1Vertices, motor2Vertices, motor3Vertices, motor4Vertices, motor5Vertices)

boardEdges=(
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 6),
    (4, 7),
    (4, 5),
    (6, 5),
    (6, 7),
    (3, 7),
    (1, 5)
)

motorEdges=(
    (0, 1),
    (0, 3),
    (0, 4),
    (1, 4),
    (1, 2),
    (2, 4),
    (2, 3),
    (3, 4)

)

coordinateSystemEdges=(
    (0, 1),
    (0, 2),
    (0, 3),
    (1, 4),
    (1, 5),
    (2, 6),
    (2, 7),
    (3, 8),
    (3, 9)
)

boardSurfaces=(
    (0, 1, 2, 3),
    (0, 1, 5, 4),
    (4, 5, 6, 7),
    (2, 3, 6, 7),
    (4, 6, 3, 0),
    (1, 2, 6, 5),
)

motorSurfaces=(
    (0, 1, 2, 3),
    (0, 1, 4),
    (2, 1, 4),
    (2, 3, 4),
    (0, 3, 4),
)

class Simulation3D(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.currentAngles = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}  # YPR angles -> OZ, OY', OX''
        self.currentDepth = 0.0
        self.currentMotorsPower = [100.0, 100.0, 100.0, 100.0, 100.0]  # testing values

    def setCurrentAngles(self, angles):
        #print(angles)
        roll = angles['roll']
        pitch = angles['pitch']
        yaw = angles['yaw']
        self.currentAngles['yaw'] = yaw
        self.currentAngles['pitch'] = pitch
        self.currentAngles['roll'] = roll

    def setCurrentDepth(self, depth):
        self.currentDepth = depth

    def setCurrentMotorsPower(self, powers):
        for p in range(5):
            self.currentDepth[p] = powers[p]

    def drawBoard(self):
        glLineWidth(3)  # 3px
        glBegin(GL_QUADS)
        glColor3f(1, 1, 0)
        for surface in boardSurfaces:
            for vertex in surface:
                vert = self.rotate({'yaw': 90, 'pitch': 0, 'roll': -90}, boardVertices[vertex])
                glVertex3fv(self.imitateDiving(self.currentDepth, self.rotate(self.currentAngles, vert)))
        glEnd()
        glBegin(GL_LINES)
        glColor3f(0.5,0.5,0)
        for edge in boardEdges:
            for vertex in edge:
                vert = self.rotate({'yaw': 90, 'pitch': 0, 'roll': -90}, boardVertices[vertex])
                glVertex3fv(self.imitateDiving(self.currentDepth, self.rotate(self.currentAngles, vert)))
        glEnd()

    def drawMotors(self, motorIndex):
        glLineWidth(3)  # 3px
        glBegin(GL_QUADS)
        glColor3f(self.currentMotorsPower[motorIndex] / MAX_MOTORS_POWER, 0.001, 0.001)
        for surface in motorSurfaces:
            for vertex in surface:
                vert = self.rotate({'yaw': 90, 'pitch': 0, 'roll': -90}, motorsVertices[motorIndex][vertex])
                glVertex3fv(self.imitateDiving(self.currentDepth, self.rotate(self.currentAngles, vert)))
        glEnd()
        glBegin(GL_LINES)
        glColor3f(0.3, 0.2, 0.2)
        for edge in motorEdges:
            for vertex in edge:
                vert = self.rotate({'yaw': 90, 'pitch': 0, 'roll': -90}, motorsVertices[motorIndex][vertex])
                glVertex3fv(self.imitateDiving(self.currentDepth, self.rotate(self.currentAngles, vert)))
        glEnd()


    def drawCoordinateSystem(self):
        glLineWidth(3)  # 3px
        glBegin(GL_LINES)

        for edge in coordinateSystemEdges:
            if edge == (0, 1):
                glColor3f(1, 0, 0)
            elif edge == (0, 2):
                glColor3f(0, 1, 0)
            elif edge == (0, 3):
                glColor3f(0, 0, 1)
            else:
                glColor3f(1, 1, 1)
            for vertex in edge:
                glVertex3fv(coordinateSystemVertices[vertex])
        glEnd()

    def rotate(self, angles, vector):

        angles = dict(map(lambda x: (x, math.radians(angles[x])), angles))
        # for Euler angles : but I changed list of angles to dictionary
        # rotMatrixEA = np.array(
        #     [[math.cos(angles[2]) * math.cos(angles[0]) - math.sin(angles[2]) * math.cos(angles[1]) * math.sin(angles[0]),
        #       math.cos(angles[2]) * math.sin(angles[0]) + math.sin(angles[2]) * math.cos(angles[1]) * math.cos(angles[0]),
        #       math.sin(angles[2]) * math.sin(angles[1])
        #       ],
        #      [-math.sin(angles[2]) * math.cos(angles[0]) - math.cos(angles[2]) * math.cos(angles[1]) * math.sin(angles[0]),
        #       -math.sin(angles[2]) * math.sin(angles[0]) + math.cos(angles[2]) * math.cos(angles[1]) * math.cos(angles[0]),
        #       math.cos(angles[2]) * math.sin(angles[1])
        #       ],
        #      [math.sin(angles[1]) * math.sin(angles[0]),
        #       -math.sin(angles[1]) * math.cos(angles[0]),
        #       math.cos(angles[1])
        #       ]], np.float_)

        rotMatrixYPR = np.array(
            [[math.cos(angles['yaw']) * math.cos(angles['pitch']),
              math.cos(angles['yaw']) * math.sin(angles['pitch']) * math.sin(angles['roll']) - math.sin(
                  angles['yaw']) * math.cos(angles['roll']),
              math.cos(angles['yaw']) * math.sin(angles['pitch']) * math.cos(angles['roll']) + math.sin(
                  angles['yaw']) * math.sin(angles['roll']),
              ],
             [math.sin(angles['yaw']) * math.cos(angles['pitch']),
              math.sin(angles['yaw']) * math.sin(angles['pitch']) * math.sin(angles['roll']) + math.cos(
                  angles['yaw']) * math.cos(angles['roll']),
              math.sin(angles['yaw']) * math.sin(angles['pitch']) * math.cos(angles['roll']) - math.cos(
                  angles['yaw']) * math.sin(angles['roll']),
              ],
             [-math.sin(angles['pitch']),
              math.cos(angles['pitch']) * math.sin(angles['roll']),
              math.cos(angles['pitch']) * math.cos(angles['roll']),
              ]], np.float_)

        vectorMatrix = np.asarray(vector, np.float_)
        #vectorMatrix[1], vectorMatrix[2] = vectorMatrix[2], vectorMatrix[1]
        #print(vector)
        #print(vectorMatrix)
        #x = np.transpose(vectorMatrix)
        #y = np.transpose(rotMatrix)
        return rotMatrixYPR.dot(vectorMatrix)

    def imitateDiving(self, depth, vertex):
        vert = (vertex[0], vertex[1], vertex[2] + depth)
        return vert

    def run(self):
        pygame.init()
        display = (1000, 600)
        surface = pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('OKON Submarine - testing field 2019')

        gluPerspective(45, (display[0]/display[1]), 0.1, 100)
        glTranslatef(0, 0, -50)

        glTranslatef(-15, 10, 0)
        glRotatef(-25, 0, 1, 0)
        glRotatef(25, 1, 0, 0)
        glRotatef(-35, 0, 1, 0)
        glRotatef(10, 1, 1, 0)
        #glRotatef(10, 0, 0, 1)
        glRotatef(30, 0, 1, 0)
        glRotatef(-30, 0, 0, 1)
        glRotatef(90, 1, 0, 0)

        clock = pygame.time.Clock()
        while True:
            clock.tick(60)  #60fps
            #self.currentAngles['yaw']+=1
            #self.currentAngles['pitch']+=5
            #self.currentAngles['roll']+=5
            #self.currentDepth+=1
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
            #glRotatef(2, 1, 1, 1)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

            self.drawBoard()
            for x in range(5):
                self.drawMotors(x)
            self.drawCoordinateSystem()
            pygame.display.flip()


#s = Simulation3D()
#s.start()
