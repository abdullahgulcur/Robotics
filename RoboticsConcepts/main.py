import math
import numpy as np
from vpython import *

# Developed by ABDULLAH GULCUR with the help of Dr. MEHMET KADIR BARAN
# Import all the libraries before running program
# This program demonstrates Articulate Robot
# There are specific calculations in the project
# 1-) Forward and Inverse kinematic calculations
# 2-) Jacobian calculations
# 3-) Wrist calculations
# If there is any problem, contact abdullahgulcur2@gmail.com

# drawing x y z axis
axisRadius = 0.01
axisLen = 5
xaxis = cylinder(pos=vec(0,0,0), axis=vec(axisLen,0,0), radius=axisRadius, color=color.red)
yaxis = cylinder(pos=vec(0,0,0), axis=vec(0,axisLen,0), radius=axisRadius, color=color.green)
zaxis = cylinder(pos=vec(0,0,0), axis=vec(0,0,axisLen), radius=axisRadius, color=color.blue)

# all the parameters for the articulate robot
cylindericalJointRadius = 0.2
linkRadius = 0.025
cylindericalJointHeight = 0.5
endEffectorRadius = 0.04
wristJointHeight = 0.07
wristLengthBetweenJoints = 0.15
wristJointRadius = 0.045
wristLinkRadius = 0.015
sprayHeight = 0.2
sprayRadius = 0.08

class Articulate:
    def __init__(self, a1, a2, a3):

        # all the angles for the robot including wrist
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.theta4 = 0
        self.theta5 = 0
        self.theta6 = 0

        # end effector position, velocity and angular velocities
        self.posX = 0
        self.posY = 0
        self.posZ = 0
        self.velX = 0
        self.velY = 0
        self.velZ = 0
        self.velTheta1 = 0
        self.velTheta2 = 0
        self.velTheta3 = 0

        # link lengths
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3

        # all the rendered components are initialized here
        self.joint0 = cylinder(axis=vec(0, 0, cylindericalJointHeight),radius=cylindericalJointRadius, color=color.yellow)
        self.link0 = cylinder(axis=vec(0, 0, self.a1), radius=linkRadius, color=color.white)
        self.joint1 = cylinder(axis=vec(0, cylindericalJointHeight, 0), radius=cylindericalJointRadius, color=color.yellow)
        self.link1 = cylinder(axis=vec(self.a2, 0, 0), radius=linkRadius, color=color.white)
        self.joint2 = cylinder(axis=vec(0, cylindericalJointHeight, 0), radius=cylindericalJointRadius, color=color.yellow)
        self.link2 = cylinder(axis=vec(a3, 0, 0), radius=linkRadius, color=color.white)
        self.wristJoint0 = cylinder(axis=vec(0, wristJointHeight, 0), radius=wristJointRadius, color=color.yellow)
        self.wristLink0 = cylinder(axis=vec(0, wristLengthBetweenJoints, 0), radius=wristLinkRadius, color=color.white)
        self.wristJoint1 = cylinder(axis=vec(0, wristJointHeight, 0), radius=wristJointRadius, color=color.yellow)
        self.wristLink1 = cylinder(axis=vec(wristLengthBetweenJoints, 0, 0), radius=wristLinkRadius, color=color.white)
        self.wristJoint2 = cylinder(axis=vec(0, wristJointHeight, 0), radius=wristJointRadius, color=color.yellow)
        self.spray = box(size=vec(sprayHeight, sprayHeight / 2, sprayHeight / 2), color=color.red, make_trail=True, trail_type='points', interval=10, retain=50)

    # all the angles are calculated according to inverse kinematics of articulate robot
    # formulas are taken from Angela Sodemann's youtube channel video about ik articulate robot
    def SetInverseKinematicAngles(self, X, Y, Z):
        self.theta1 = math.atan(Y / X)
        r_1 = math.sqrt(X**2 + Y**2)
        r_2 = Z - self.a1
        r_3 = math.sqrt(r_1**2 + r_2**2)
        fi_2 = math.atan(r_2 / r_1)
        fi_1 = math.acos((self.a3 ** 2 - self.a2 ** 2 - r_3 ** 2) / ((-2) * self.a2 * r_3))
        self.theta2 = fi_2 - fi_1
        fi_3 = math.acos((r_3 ** 2 - self.a2 ** 2 - self.a3 ** 2) / ((-2) * self.a2 * self.a3))
        self.theta3 = math.pi - fi_3

    # jacobian matrix are calculated here
    def GetJacobianMatrix(self, theta1, theta2, theta3):
        temp = np.dot(np.identity(3), np.transpose([0,0,1]))
        J_0_0 = np.cross(temp, self.GetD0_3(theta1, theta2, theta3))

        temp = np.dot(self.GetR0_1(theta1), np.transpose([0,0,1]))
        J_0_1 = np.cross(temp, np.subtract(self.GetD0_3(theta1, theta2, theta3), self.GetD0_1()))

        temp = np.dot(self.GetR0_2(theta1, theta2), np.transpose([0, 0, 1]))
        J_0_2 = np.cross(temp, np.subtract(self.GetD0_3(theta1, theta2, theta3), self.GetD0_2(theta1, theta2)))

        J = [[J_0_0[0], J_0_1[0], J_0_2[0]],
             [J_0_0[1], J_0_1[1], J_0_2[1]],
             [J_0_0[2], J_0_1[2], J_0_2[2]]]
        return J

    # all the angles are updated here
    def SetAllTheAnglesFromCalculations(self, w, t, dt):

        # set end effectors velocity
        self.SetEndEffectorVelByTime(w, t)

        # jacobian inverse to calculate angle speeds
        jacobianInverse = np.linalg.inv(self.GetJacobianMatrix(self.theta1, self.theta2, self.theta3))
        angleVel = np.dot(jacobianInverse, np.transpose(self.GetEndEffectorVel()))

        # angle_new += angle_velocity * dt
        self.theta1 += angleVel[0] * dt
        self.theta2 += angleVel[1] * dt
        self.theta3 += angleVel[2] * dt

        # hands orientation is same as first frame
        # R0_3 * R3_6 = R0_6 = I
        # inverse(R0_3) * R0_3 * R3_6 = inverse(R0_3) * I
        # R3_6 = inverse(R0_3)
        R3_6 = self.GetR3_6_ForIdentity(self.GetR0_3(self.theta1, self.theta2, self.theta3))

        # wrist angles are calculated according to R3_6 matrix (long matrix that is presented in classroom but different version)
        self.theta5 = math.asin(R3_6[2][2])
        self.theta4 = math.asin(R3_6[1][2]/(math.cos(self.theta5)))
        self.theta6 = math.asin(R3_6[2][0]/(math.cos(self.theta5)))

        # rotate all the angles in robot
        self.Rotate(self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6)

    # sets end effectors position according to time (circular motion)
    def SetEndEffectorPosByTime(self, x_0, y_0, z_0, r, w, t):
        self.posX = x_0 + r * math.cos(w*t)
        self.posY = y_0 + r * math.sin(w*t)
        self.posZ = z_0
        self.SetInverseKinematicAngles(self.posX, self.posY, self.posZ)
        self.Rotate(self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6)

    # sets end effectors velocity according to time (circular motion)
    def SetEndEffectorVelByTime(self, w, t):
        self.velX = - w * math.sin(w*t)
        self.velY = w * math.cos(w*t)
        self.velZ = 0

    # gets end effectors velocity
    def GetEndEffectorVel(self):
        vel = [self.velX, self.velY, self.velZ]
        return vel

    # rotate all the angles in robot
    def Rotate(self, theta0, theta1, theta2, theta4, theta5, theta6):

        # gets all the rotation matrices
        r0_1 = self.GetR0_1(theta0)
        d0_1 = self.GetD0_1()
        r0_2 = self.GetR0_2(theta0, theta1)
        r0_3 = self.GetR0_3(theta0, theta1, theta2)
        r0_4 = self.GetR0_4(theta0, theta1, theta2, theta4)
        r0_5 = self.GetR0_5(theta0, theta1, theta2, theta4, theta5)
        r0_6 = self.GetR0_6(theta0, theta1, theta2, theta4, theta5, theta6)

        # gets all the distance matrices
        d0_2 = self.GetD0_2(theta0, theta1)
        d0_3 = self.GetD0_3(theta0, theta1, theta2)
        d0_4 = self.GetD0_4(theta0, theta1, theta2, theta4)
        d0_5 = self.GetD0_5(theta0, theta1, theta2, theta4, theta5)
        d0_6 = self.GetD0_6(theta0, theta1, theta2, theta4, theta5, theta6)

        # after this part, all the drawable elements axis and position values are updated
        # most difficult part of the project

        newAxis_joint1 = np.matmul(r0_1, ([0], [0], [cylindericalJointHeight]))
        newAxis_link1 = np.matmul(r0_2, ([self.a2], [0], [0]))
        self.joint1.axis = vec(newAxis_joint1[0], newAxis_joint1[1], newAxis_joint1[2])
        self.link1.axis = vec(newAxis_link1[0], newAxis_link1[1], newAxis_link1[2])
        self.joint1.pos = vec(d0_1[0] + newAxis_joint1[0] * (-0.5), d0_1[1] + newAxis_joint1[1] * (-0.5), d0_1[2])
        self.link1.pos = vec(d0_1[0], d0_1[1], d0_1[2])

        newAxis_joint2 = np.matmul(r0_2, ([0], [0], [cylindericalJointHeight]))
        newAxis_link2 = np.matmul(r0_3, ([0], [0], [self.a3]))
        self.joint2.axis = vec(newAxis_joint2[0], newAxis_joint2[1], newAxis_joint2[2])
        self.link2.axis = vec(newAxis_link2[0], newAxis_link2[1], newAxis_link2[2])
        self.joint2.pos = vec(d0_2[0] + newAxis_joint2[0] * (-0.5), d0_2[1] + newAxis_joint2[1] * (-0.5), d0_2[2])
        self.link2.pos = vec(d0_2[0], d0_2[1], d0_2[2])

        newAxis_wristJoint0 = np.matmul(r0_3, ([0], [0], [wristJointHeight]))
        newAxis_wristLink0 = np.matmul(r0_3, ([0], [0], [wristLengthBetweenJoints]))
        self.wristJoint0.axis = vec(newAxis_wristJoint0[0], newAxis_wristJoint0[1], newAxis_wristJoint0[2])
        self.wristJoint0.pos = vec(d0_3[0], d0_3[1], d0_3[2])
        self.wristLink0.axis = vec(newAxis_wristLink0[0], newAxis_wristLink0[1], newAxis_wristLink0[2])
        self.wristLink0.pos = vec(d0_3[0], d0_3[1], d0_3[2])

        newAxis_wristJoint1 = np.matmul(r0_4, ([0], [0], [wristJointHeight]))
        self.wristJoint1.axis = vec(newAxis_wristJoint1[0], newAxis_wristJoint1[1], newAxis_wristJoint1[2])
        self.wristJoint1.pos = vec(d0_4[0] + newAxis_wristJoint1[0] * (-0.5), d0_4[1] + newAxis_wristJoint1[1] * (-0.5), d0_4[2] + newAxis_wristJoint1[2] * (-0.5))

        newAxis_wristLink1 = np.matmul(r0_5, ([0], [0], [2 * wristLengthBetweenJoints - wristJointHeight - wristLinkRadius]))
        self.wristLink1.axis = vec(newAxis_wristLink1[0], newAxis_wristLink1[1], newAxis_wristLink1[2])
        self.wristLink1.pos = vec(d0_4[0], d0_4[1], d0_4[2])

        newAxis_wristJoint2 = np.matmul(r0_5, ([0], [0], [wristJointHeight]))
        self.wristJoint2.axis = vec(newAxis_wristJoint2[0], newAxis_wristJoint2[1], newAxis_wristJoint2[2])
        self.wristJoint2.pos = vec(d0_5[0] - newAxis_wristJoint2[0], d0_5[1] - newAxis_wristJoint2[1], d0_5[2] - newAxis_wristJoint2[2])

        newAxis_spray = np.matmul(r0_6, ([0], [0], [sprayHeight]))
        self.spray.axis = vec(newAxis_spray[0], newAxis_spray[1], newAxis_spray[2])
        self.spray.pos = vec(d0_6[0], d0_6[1], d0_6[2] + newAxis_spray[2] / 2)

    def GetR0_1(self, theta0):
        rotateZMatrix = ([math.cos(theta0), -math.sin(theta0), 0],
                [math.sin(theta0), math.cos(theta0), 0],
                [0, 0, 1])

        basisChange = ([1, 0, 0],
                [0, 0, -1],
                [0, 1, 0])

        r0_1 = np.matmul(rotateZMatrix, basisChange)
        return r0_1

    def GetR1_2(self, theta1):
        rotateZMatrix = ([math.cos(theta1), -math.sin(theta1), 0],
                [math.sin(theta1), math.cos(theta1), 0],
                [0, 0, 1])

        r1_2 = np.matmul(rotateZMatrix, np.identity(3))
        return r1_2

    def GetR2_3(self, theta2):
        rotateZMatrix = ([math.cos(theta2), -math.sin(theta2), 0],
                [math.sin(theta2), math.cos(theta2), 0],
                [0, 0, 1])

        basisChange = ([0, 0, 1],
                       [1, 0, 0],
                       [0, 1, 0])

        r2_3 = np.matmul(rotateZMatrix, basisChange)
        return r2_3

    def GetR3_4(self, theta4):
        rotateZMatrix = ([math.cos(theta4), -math.sin(theta4), 0],
                [math.sin(theta4), math.cos(theta4), 0],
                [0, 0, 1])

        basisChange = ([1, 0, 0],
                       [0, 0, -1],
                       [0, 1, 0])

        r3_4 = np.matmul(rotateZMatrix, basisChange)
        return r3_4

    def GetR4_5(self, theta5):
        rotateZMatrix = ([math.cos(theta5), -math.sin(theta5), 0],
                [math.sin(theta5), math.cos(theta5), 0],
                [0, 0, 1])

        basisChange = ([0, 0, 1],
                       [1, 0, 0],
                       [0, 1, 0])

        r4_5 = np.matmul(rotateZMatrix, basisChange)
        return r4_5

    def GetR5_6(self, theta6):
        rotateZMatrix = ([math.cos(theta6), -math.sin(theta6), 0],
                [math.sin(theta6), math.cos(theta6), 0],
                [0, 0, 1])

        r5_6 = np.matmul(rotateZMatrix, np.identity(3))
        return r5_6

    def GetR0_2(self, theta0, theta1):
        r0_2 = np.matmul(self.GetR0_1(theta0), self.GetR1_2(theta1))
        return r0_2

    def GetR0_3(self, theta0, theta1, theta2):
        r0_3 = np.matmul(self.GetR0_2(theta0, theta1), self.GetR2_3(theta2))
        return r0_3

    def GetR0_4(self, theta0, theta1, theta2, theta3):
        r0_4 = np.matmul(self.GetR0_3(theta0, theta1, theta2), self.GetR3_4(theta3))
        return r0_4

    def GetR0_5(self, theta0, theta1, theta2, theta3, theta4):
        r0_5 = np.matmul(self.GetR0_4(theta0, theta1, theta2, theta3), self.GetR4_5(theta4))
        return r0_5

    def GetR0_6(self, theta0, theta1, theta2, theta3, theta4, theta5):
        r0_6 = np.matmul(self.GetR0_5(theta0, theta1, theta2, theta3, theta4), self.GetR5_6(theta5))
        return r0_6

    def GetR3_6_ForIdentity(self, R0_3):
        R3_6 = np.linalg.inv(R0_3)
        return R3_6

    def GetD0_1(self):
        d0_1 = ([0, 0, self.a1])
        return d0_1

    def GetD1_2(self, theta1):
        d1_2= ([self.a2 * math.cos(theta1), self.a2 * math.sin(theta1), 0])
        return d1_2

    def GetD2_3(self, theta2):
        d2_3 = ([self.a3 * math.cos(theta2), self.a3 * math.sin(theta2), 0])
        return d2_3

    def GetD3_4(self, theta4):
        d3_4 = ([0, 0, wristLengthBetweenJoints])
        return d3_4

    def GetD4_5(self, theta5):
        d4_5 = ([wristLengthBetweenJoints * math.cos(theta5), wristLengthBetweenJoints * math.sin(theta5), 0])
        return d4_5

    def GetD5_6(self, theta6):
        d5_6 = ([0, 0, wristLengthBetweenJoints - wristJointHeight - wristJointRadius])
        return d5_6

    def GetD0_2(self, theta1, theta2):
        h0_2 = self.GetH0_2(theta1, theta2)
        d0_2 = ([h0_2[0][3], h0_2[1][3], h0_2[2][3]])
        return  d0_2

    def GetD0_3(self, theta1, theta2, theta3):
        h0_3 = self.GetH0_3(theta1, theta2, theta3)
        d0_3 = ([h0_3[0][3], h0_3[1][3], h0_3[2][3]])
        return d0_3

    def GetD0_4(self, theta1, theta2, theta3, theta4):
        h0_4 = self.GetH0_4(theta1, theta2, theta3, theta4)
        d0_4 = ([h0_4[0][3], h0_4[1][3], h0_4[2][3]])
        return d0_4

    def GetD0_5(self, theta1, theta2, theta3, theta4, theta5):
        h0_5 = self.GetH0_5(theta1, theta2, theta3, theta4, theta5)
        d0_5 = ([h0_5[0][3], h0_5[1][3], h0_5[2][3]])
        return d0_5

    def GetD0_6(self, theta1, theta2, theta3, theta4, theta5, theta6):
        h0_6 = self.GetH0_6(theta1, theta2, theta3, theta4, theta5, theta6)
        d0_6 = ([h0_6[0][3], h0_6[1][3], h0_6[2][3]])
        return d0_6

    def GetH0_1(self, theta0):
        r0_1 = self.GetR0_1(theta0)
        d0_1 = self.GetD0_1()

        h0_1 = ([r0_1[0][0], r0_1[0][1], r0_1[0][2], d0_1[0]],
                [r0_1[1][0], r0_1[1][1], r0_1[1][2], d0_1[1]],
                [r0_1[2][0], r0_1[2][1], r0_1[2][2], d0_1[2]],
                [0, 0, 0, 1])

        return h0_1

    def GetH1_2(self, theta1):
        r1_2 = self.GetR1_2(theta1)
        d1_2 = self.GetD1_2(theta1)

        h1_2 = ([r1_2[0][0], r1_2[0][1], r1_2[0][2], d1_2[0]],
                [r1_2[1][0], r1_2[1][1], r1_2[1][2], d1_2[1]],
                [r1_2[2][0], r1_2[2][1], r1_2[2][2], d1_2[2]],
                [0, 0, 0, 1])

        return h1_2

    def GetH2_3(self, theta2):
        r2_3 = self.GetR2_3(theta2)
        d2_3 = self.GetD2_3(theta2)

        h2_3 = ([r2_3[0][0], r2_3[0][1], r2_3[0][2], d2_3[0]],
                [r2_3[1][0], r2_3[1][1], r2_3[1][2], d2_3[1]],
                [r2_3[2][0], r2_3[2][1], r2_3[2][2], d2_3[2]],
                [0, 0, 0, 1])

        return h2_3

    def GetH3_4(self, theta4):
        r3_4 = self.GetR3_4(theta4)
        d3_4 = self.GetD3_4(theta4)

        h3_4 = ([r3_4[0][0], r3_4[0][1], r3_4[0][2], d3_4[0]],
                [r3_4[1][0], r3_4[1][1], r3_4[1][2], d3_4[1]],
                [r3_4[2][0], r3_4[2][1], r3_4[2][2], d3_4[2]],
                [0, 0, 0, 1])

        return h3_4

    def GetH4_5(self, theta5):
        r4_5 = self.GetR4_5(theta5)
        d4_5 = self.GetD4_5(theta5)

        h4_5 = ([r4_5[0][0], r4_5[0][1], r4_5[0][2], d4_5[0]],
                [r4_5[1][0], r4_5[1][1], r4_5[1][2], d4_5[1]],
                [r4_5[2][0], r4_5[2][1], r4_5[2][2], d4_5[2]],
                [0, 0, 0, 1])

        return h4_5

    def GetH5_6(self, theta6):
        r5_6 = self.GetR5_6(theta6)
        d5_6 = self.GetD5_6(theta6)

        h5_6 = ([r5_6[0][0], r5_6[0][1], r5_6[0][2], d5_6[0]],
                [r5_6[1][0], r5_6[1][1], r5_6[1][2], d5_6[1]],
                [r5_6[2][0], r5_6[2][1], r5_6[2][2], d5_6[2]],
                [0, 0, 0, 1])

        return h5_6

    def GetH0_2(self, theta1, theta2):
        h0_2 = np.matmul(self.GetH0_1(theta1), self.GetH1_2(theta2))
        return h0_2

    def GetH0_3(self, theta1, theta2, theta3):
        h0_3 = np.matmul(self.GetH0_2(theta1, theta2), self.GetH2_3(theta3))
        return h0_3

    def GetH0_4(self, theta1, theta2, theta3, theta4):
        h0_4 = np.matmul(self.GetH0_3(theta1, theta2, theta3), self.GetH3_4(theta4))
        return h0_4

    def GetH0_5(self, theta1, theta2, theta3, theta4, theta5):
        h0_5 = np.matmul(self.GetH0_4(theta1, theta2, theta3, theta4), self.GetH4_5(theta5))
        return h0_5

    def GetH0_6(self, theta1, theta2, theta3, theta4, theta5, theta6):
        h0_6 = np.matmul(self.GetH0_5(theta1, theta2, theta3, theta4, theta5), self.GetH5_6(theta6))
        return h0_6


def main():
    articulate = Articulate(1.5,1.5,1.5)

    t = 0
    w = 0.01
    dt = 0.8

    articulate.SetEndEffectorPosByTime(1.5, 1.5, 2, 0.75, w, t)

    while True:
        rate(60)
        articulate.SetAllTheAnglesFromCalculations(w, t, dt)
        t += 1

main()