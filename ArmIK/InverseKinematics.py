#!/usr/bin/env python3
# encoding: utf-8
# Inverse kinematics of the 4-DOF manipulator: 
# Given the corresponding coordinates (X, Y, Z) and the pitch angle,
# calculate the rotation angle of each joint
# 2020/07/20 Aiden

import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

class IK:
    # Servo counts from bottom to top
    # Common parameters, that is, the link parameters of the 4-DOF manipulator

    # The distance from the center of the robotic arm chassis
    # to the center axis of the second steering gear is 6.10cm
    # The distance from the second servo to the third servo is 10.16cm
    # The distance from the third servo to the fourth servo is 9.64cm
    # No specific assignment is made here, and reassignment is performed according to the selection during initialization
    l1 = 6.10   
    l2 = 10.16 
    l3 = 9.64  
    l4 = 0.00

    # Air pump specific parameters
    # The distance from the fourth servo to the top of the suction nozzle is 4.70cm
    # The distance from the top of the suction nozzle to the suction nozzle is 4.46cm
    l5 = 4.70  
    l6 = 4.46 
    alpha = degrees(atan(l6 / l5))  # Calculate the angle between l5 and l4

    def __init__(self, arm_type): # According to different grippers, adaptation parameters
        self.arm_type = arm_type
        if self.arm_type == 'pump': # If it is an air pump type robotic arm.
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))  # 4th servo to suction nozzle as 4th link
        elif self.arm_type == 'arm':
            # The distance from the fourth servo to the end of the robotic arm is 16.6cm, 
            # and the end of the robotic arm is when the claw is fully closed
            self.l4 = 16.65  

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4, L5=l5, L6=l6):
        # Change the link length of the robotic arm, 
        # in order to adapt to different lengths of robotic arms of the same structure
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4
        self.l5 = L5
        self.l6 = L6
        if self.arm_type == 'pump':
            self.l4 = sqrt(pow(self.l5, 2) + pow(self.l6, 2))
            self.alpha = degrees(atan(self.l6 / self.l5))

    def getLinkLength(self):
        # Get the currently set link length
        if self.arm_type == 'pump':
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4, "L5":self.l5, "L6":self.l6}
        else:
            return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # Given the specified coordinates and pitch angle,
        # returns the angle that each joint should rotate, or False if there is no solution.
        # coordinate_data is the coordinate of the end of the gripper,
        # the coordinate unit is cm, which is passed in as a tuple, for example (0, 5, 10).
        # Alpha is the angle between the gripper and the horizontal plane, in degrees.

        # Let the end of the gripper be P(X, Y, Z), the coordinate origin is O,
        # the origin is the projection of the center of the gimbal on the ground,
        # and the projection of point P on the ground is P_
        #
        # The intersection of l1 and l2 is A, the intersection of l2 and l3 is B,
        # and the intersection of l3 and l4 is C
        #
        # CD is perpendicular to PD, CD is perpendicular to z-axis, 
        # then the pitch angle Alpha is the angle between DC and PC, 
        # AE is perpendicular to DP_, and E is on DP_, 
        # CF is perpendicular to AE, and F is on AE
        #
        # Included angle representation: 
        # For example, the included angle between AB and BC is represented as ABC
        X, Y, Z = coordinate_data
        if self.arm_type == 'pump':
            Alpha -= self.alpha
        # Find the base rotation angle
        theta6 = degrees(atan2(Y, X))
 
        P_O = sqrt(X*X + Y*Y) # P_ distance from origin O
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha))  # The sign of PD is the same as that of pitch angle.
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        if round(CF, 4) < -self.l1:
            logger.debug('height below 0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): # The sum of two sides is less than the third side
            logger.debug('Cannot form this configuration, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        # asking theta 4 and using cosine Theorem.
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) 
        if abs(cos_ABC) > 1:
            logger.debug('Cannot form this configuration, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) # Inverse trigonometry to find radians
        theta4 = 180.0 - degrees(ABC)

        # asking theta 5 and using cosine Theorem.
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4)
        if abs(cos_BAC) > 1:
            logger.debug('Cannot form this configuration, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        #求theta3
        theta3 = Alpha - theta5 + theta4
        if self.arm_type == 'pump':
            theta3 += self.alpha

        # Returns a dictionary of angles when there is a solution
        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6}
            
if __name__ == '__main__':
    ik = IK('arm')
    ik.setLinkLength(L1=ik.l1 + 0.89, L4=ik.l4 - 0.3)
    print('Link length: ', ik.getLinkLength())
    print(ik.getRotationAngle((0, 0, ik.l1 + ik.l2 + ik.l3 + ik.l4), 90))
