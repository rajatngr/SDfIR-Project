import math
import numpy as np
from copy import copy
import sys
import os

from .CalculateWCP import calculateWCP
from ...postypes.configuration import configuration
from ...postypes import SolutionSets


# TODO: add Type hints
class IKPosSolver:
    """
    (Singleton) solver class for inverse kinematics for the first 3 angles of the robot.
    This does not get used directly in inv_kinematics.py; instead, the orientation part uses it.
    """
    # define robot constants
    d6 = 215
    a_0 = 330
    a_1 = 645
    a_2 = 1150
    a_3 = 1220
    a_4 = 115
    c = math.sqrt((a_4 ** 2) + (a_3 ** 2))
    q = math.atan(a_4 / a_3)

    def __init__(self, TCP=None):
        """ Constructor with optional early TCP configuration """

        # initialize instance variables for angle calculations
        self.d  = None
        self.b  = None
        self.bb = None
        self.beta   = None
        self.alpha  = None
        self.delta  = None
        self.beta2  = None
        self.alpha2 = None

        # initialize set of solutions
        self.Solutions = SolutionSets.InvKinPosSet()

        # check if TCP was passed; if so: solve already
        if TCP is not None:
            self.setup_WCP(TCP)
            self.solve()

        # if we don't have a TCP, initialize to None and dont solve
        else:
            self.TCP = None
            self.Pw_x, self.Pw_y, self.Pw_z = None, None, None

    def setup_WCP(self, TCP):
        self.TCP = TCP
        self.Pw_x, self.Pw_y, self.Pw_z = calculateWCP(self.TCP, self.d6)

    def solve(self, TCP=None):

        # if we get a (new) TCP, update our variables
        if TCP is not None:
            self.TCP = TCP
            self.Pw_x, self.Pw_y, self.Pw_z = calculateWCP(self.TCP, self.d6)

        # otherwise, check if it was initialized in the constructor
        else:
            assert self.TCP is not None

        # big triangle
        self.d = math.sqrt(((self.Pw_x) ** 2) + (self.Pw_y ** 2))
        self.b = math.sqrt(((self.d - self.a_0) ** 2) + ((self.Pw_z - self.a_1) ** 2))  # forward facing
        self.bb = math.sqrt(((self.d + self.a_0) ** 2) + ((self.Pw_z - self.a_1) ** 2))  # backward facing

        # angles
        # forward facing solutions
        self.beta = math.acos((self.d - self.a_0) / self.b)
        self.alpha = math.acos(((self.b ** 2) + (self.a_2 ** 2) - (self.c ** 2)) / (2 * self.b * self.a_2))

        # backward facing solutions
        self.delta = math.acos(((self.a_2 ** 2) + (self.c ** 2) - (self.b ** 2)) / (2 * self.a_2 * self.c))
        self.beta2 = math.acos((self.d + self.a_0) / self.bb)
        self.alpha2 = math.acos(((self.bb ** 2) + (self.a_2 ** 2) - (self.c ** 2)) / (2 * self.bb * self.a_2))

        # First 4 Solutions
        self.CalculateFfElbowUp()
        self.CalculateFfElbowDn()
        self.CalculateBfElbowUp()
        self.CalculateBfElbowDn()

        #
        self.Solutions.bfElbowUpN = copy(self.Solutions.bfElbowUpP)
        self.Solutions.bfElbowUpN[0] = self.Solutions.ffElbowUp[0] + 180

        #
        self.Solutions.bfElbowDnN = copy(self.Solutions.bfElbowDnP)
        self.Solutions.bfElbowDnN[0] = self.Solutions.ffElbowDn[0] + 180

    def CalculateFfElbowUp(self):
        # TODO: rename all of these functions to be lowercase and snake_case
        # calculate theta_1
        theta_1 = -np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))
        theta_1_1 = theta_1 - 360
        theta_1_2 = theta_1 + 360

        # calculate theta_2
        theta_2 = -math.degrees(self.alpha + self.beta)

        # calculate theta_3
        self.m = math.acos(((self.c ** 2) + (self.b ** 2) - (self.a_2 ** 2)) / (2 * self.b * self.c))
        self.s = 180 - self.alpha - (self.m + self.q)  # for triangle Pw S A3
        theta_3 = math.degrees(180 - self.s)

        # save the solution
        self.Solutions.ffElbowUp = [theta_1, theta_2, theta_3]
        self.Solutions.ffElbowUp_1 = [theta_1_1, theta_2, theta_3]
        self.Solutions.ffElbowUp_2 = [theta_1_2, theta_2, theta_3]

    def CalculateFfElbowDn(self):
        # calculate theta_1
        theta_1 = -np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))
        theta_1_1 = theta_1 - 360
        theta_1_2 = theta_1 + 360

        # calculate theta_2
        theta_2 = -math.degrees(self.beta - self.alpha)

        # calculate theta_3
        self.w = math.acos(((self.c ** 2) + (self.b ** 2) - (self.a_2 ** 2)) / (2 * self.b * self.c))
        self.e = 180 - (self.w - self.q) - self.alpha
        theta_3 = 180 - self.e
        theta_3 = math.degrees(-theta_3)

        # save the solution
        self.Solutions.ffElbowDn = [theta_1, theta_2, theta_3]
        self.Solutions.ffElbowDn_1 = [theta_1_1, theta_2, theta_3]
        self.Solutions.ffElbowDn_2 = [theta_1_2, theta_2, theta_3]

    def CalculateBfElbowUp(self):
        # calculate theta_1
        theta_1 = -(np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))) - 180

        # calculate theta_2
        theta_2 = self.beta2 + self.alpha2
        theta_2 = - math.degrees(math.pi - theta_2)

        # calculate theta_3
        self.h = math.acos(((self.c ** 2) + (self.bb ** 2) - (self.a_2 ** 2)) / (2 * self.c * self.bb)) - self.q
        self.i = 180 - math.degrees(self.h + self.alpha2)
        theta_3 = -(180 - self.i)

        # save the solution
        self.Solutions.bfElbowUpP = [theta_1, theta_2, theta_3]

    def CalculateBfElbowDn(self):
        # calculate theta_1
        theta_1 = -(np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))) - 180

        # calculate theta_2
        theta_2 = self.beta2 - self.alpha2
        theta_2 = - (180 - math.degrees(theta_2))

        # calculate theta_3
        self.x = math.acos(((self.a_2 ** 2) + (self.c ** 2) - (self.bb ** 2)) / (2 * self.a_2 * self.c))
        self.y = 180 - math.degrees(self.x)
        self.z = 180 - math.degrees(self.q) - self.y
        theta_3 = 180 - self.z

        # save the solution
        self.Solutions.bfElbowDnP = [theta_1, theta_2, theta_3]