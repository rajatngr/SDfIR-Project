import math
import numpy as np
from copy import copy
import sys
import os

from .inv_kinematics_positioning import IKPosSolver
from .inv_kinematics_orientation import IKOriSolver

from ...postypes.configuration import configuration
from ...postypes import SolutionSets
from ...postypes import SixDPos


class InvKinematics:
    def __init__(self):
        self.valid_solutions_array = None
        self.end_effector_matrix = None

    def get_inv_kinematics(self, _pos):
        values = _pos.get_position()

        # convert first 3 values to mm as we dont work with m
        values = [val * 1000 if idx <= 2 else val for idx, val in enumerate(values)]

        print("vals:", values)

        # values[3] = np.degrees(values[3])
        # values[4] = np.degrees(values[4])
        # values[5] = np.degrees(values[5])
        # print(values, input_matrix)
        # for easier testing
        solutions = []
        # if input_matrix is None:
        #     # convert sixD input to end effector matrix
        self.end_effector_matrix = self.SixDPos_2_TMat(values)
        # else:
        #     self.end_effector_matrix = input_matrix

        # #Code from MAX
        # # calculate the solutions
        # solver = IKOriSolver()
        # solver.solve(self.end_effector_matrix)
        #
        # #return the solvers configurations
        # print([config.get_configuration() for config in solver.configurations])
        # return solver.configurations

        # Code from Rajat
        # enter a input matrix
        self.mat = self.end_effector_matrix
        self.R = np.array([row[:3] for row in self.mat[:3]])
        solver = IKSolver(self.mat)
        self.all_solutions = [[solver.Solutions.bfElbowUpP], [solver.Solutions.bfElbowDnP],
                            [solver.Solutions.ffElbowUp], [solver.Solutions.ffElbowDn],
                            [solver.Solutions.bfElbowUpN], [solver.Solutions.bfElbowDnN],
                            [solver.Solutions.ffElbowUp_1], [solver.Solutions.ffElbowUp_2],
                            [solver.Solutions.ffElbowDn_1], [solver.Solutions.ffElbowDn_2]]
        #print(self.all_solutions)
        self.valid_solutions = []

        # check if 3 joint angles are in range
        for solution in self.all_solutions:
            if -185 < solution[0][0] < 185 and -140 < solution[0][1] < -5 and -120 < solution[0][2] < 168:
                # Check eblow singularity as per Pseudocode dont need for IK
                # critical_theta3 = round(90 - np.degrees(np.arctan2(1220, 115))) #why are we using round?
                # if np.isclose(solution[0][0], critical_theta3):
                #   print("Elbow Singularity Detected!")
                #   valid_solutions.append([roll-(solution[0][1]+solution[0][2]), -np.arctan2(Pw_z-645,Pw_x-330), critical_theta3])
                # else:
                self.valid_solutions.append(solution)

        # Convert the list of valid solutions to a NumPy array and print without extra formatting
        valid_solutions_array = (np.array(self.valid_solutions)).squeeze()

        # Example usage:
        arm = RobotArm()
        count = 0

        final_outputs = []

        for i in valid_solutions_array:

            # Your input for theta1, 2, 3
            end_effector = arm.forward_kinematics(valid_solutions_array[count])

            # Extract the first 3x3 submatrix
            R0_3 = end_effector[:3, :3]

            R3_6 = np.dot(np.transpose(R0_3), self.R)

            theta_5_P = np.degrees(np.arccos(-R3_6[2][2]))
            theta_5_N = -theta_5_P

            # Check wrist singularity
            if theta_5_P == 0:
                if np.isclose(R3_6[2][2], -1):
                    # Wrist singularity detected
                    theta_46_sum = np.degrees(np.arctan2(R3_6[1][0], R3_6[0][0]))
                    theta_4 = 0
                    theta_6 = theta_46_sum - theta_4
                    print("Wrist Singularity Detected:")
                    solutions.append(i[0], i[1], i[2], theta_4, theta_5_P, theta_6)
            else:
                # Calculate theta_4 from R3_6 elements
                theta_4_I = np.degrees(np.arctan2(-R3_6[1][2], -R3_6[0][2]))
                theta_4_II = np.degrees(np.arctan2(R3_6[1][2], R3_6[0][2]))
                theta_4_III = 0
                theta_4_IV = 0

                # check conditions for angles
                if 10 <= theta_4_I <= 350:
                    theta_4_III = theta_4_I - 360
                elif -350 < theta_4_I < -10:
                    theta_4_III = theta_4_I + 360

                if 10 <= theta_4_II <= 350:
                    theta_4_IV = theta_4_II - 360
                elif -350 < theta_4_II < -10:
                    theta_4_IV = theta_4_II + 360

                # Calculate theta_6 from R3_6 elements
                theta_6_I = np.degrees(np.arctan2(R3_6[2][1], R3_6[2][0]))
                theta_6_II = np.degrees(np.arctan2(-R3_6[2][1], -R3_6[2][0]))
                theta_6_III = 0
                theta_6_IV = 0

                if 10 <= theta_6_II <= 350:
                    theta_6_IV = theta_6_II - 360
                elif -350 <= theta_6_II < -10:
                    theta_6_IV = theta_6_II + 360

                # check conditions for angles
                if 10 <= theta_6_I <= 350:
                    theta_6_III = theta_6_I - 360
                elif -350 <= theta_6_I < -10:
                    theta_6_III = theta_6_I + 360

                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_III, theta_5_P, theta_6_I])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_III, theta_5_P, theta_6_III])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_I, theta_5_P, theta_6_I])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_I, theta_5_P, theta_6_III])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_II, theta_5_N, theta_6_IV])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_II, theta_5_N, theta_6_II])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_IV, theta_5_N, theta_6_IV])
                final_outputs.append(
                    [valid_solutions_array[count][0], valid_solutions_array[count][1], valid_solutions_array[count][2],
                     theta_4_IV, theta_5_N, theta_6_II])

                # Access a specific line stored in the variable
                # Change the index to access different lines
                # Print the theta 4, 5, 6
                # print(R0_3)
                # print(Sol3[count])

                # print output in all possible solutions
                # output = f"[{Sol3[count]} THETA_4 = {theta_4_I} {theta_4_II} {theta_4_III} {theta_4_IV} THETA_5 ={theta_5_I} {theta_5_II} THETA_6 ={theta_6_I} {theta_6_II} {theta_6_III} {theta_6_IV}]"
                # print(output)
                count += 1

        final_outputs = np.array(final_outputs)

        all_valid_solutions = []

        # check if all 6 joint angles are in range
        for solution in final_outputs:
            if -185 < solution[0] < 185 and -140 < solution[1] < -5 and -120 < solution[2] < 168 and -350 < solution[
                3] < 350 and -125 < solution[4] < 125 and -350 < solution[5] < 350:
                all_valid_solutions.append(solution)

        # Convert the list of valid solutions to a NumPy array and print without extra formatting
        self.valid_solutions_array = (np.array(all_valid_solutions)).squeeze()

        # get output in one line for each combination
        np.set_printoptions(linewidth=np.inf)

        # print all valid solutions
        print(self.valid_solutions_array)

        #GET OUTPUT IN CORRECT FORMAT
        for sol in np.deg2rad(self.valid_solutions_array):
            solutions.append(configuration([sol[0], sol[1], sol[2], sol[3], sol[4], sol[5]]))
        # return the solvers configurations
        return solutions



    def SixDPos_2_TMat(self, val):
        # TODO: rename, comment, non-static
        yaw = val[3]
        pitch = val[4]
        roll = val[5]
        end_effector = [val[0], val[1], val[2]]
        # Construct the matrix from 6 values for UI 3 position co-ordinates & 3 euler angles
        Endeff_matrix = np.array([[np.cos(yaw) * np.cos(pitch),
                                   np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll),
                                   np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll),
                                   end_effector[0]],
                                  [np.sin(yaw) * np.cos(pitch),
                                   np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll),
                                   np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll),
                                   end_effector[1]],
                                  [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll),
                                   end_effector[2]],
                                  [0, 0, 0, 1]])

        return Endeff_matrix


class SolutionSet:

  def __init__(self):

    self.ffElbowUp = None
    self.ffElbowDn = None

    self.bfElbowUpP = None
    self.bfElbowDnP = None

    self.bfElbowUpN = None
    self.bfElbowDnN = None

    self.ffElbowUp_1 = None
    self.ffElbowUp_2 = None

    self.ffElbowDn_1 = None
    self.ffElbowDn_2 = None


class IKSolver:

  # define robot constants
  d6 = 215
  a_0 = 330
  a_1 = 645
  a_2 = 1150
  a_3 = 1220
  a_4 = 115
  c = math.sqrt((a_4 ** 2) + (a_3 ** 2))
  q = math.atan(a_4 / a_3)

  def __init__(self, TCP):

    self.TCP = TCP
    self.Solutions = SolutionSet()
    self.CalculateWCP()

    # big triangle
    self.d = math.sqrt(((self.Pw_x)**2) + (self.Pw_y**2))
    self.b = math.sqrt(((self.d - self.a_0)**2)+((self.Pw_z-self.a_1)**2)) # forward facing

    # #Check if the point is reachable
    # if self.b > (self.a_2 + self.c):
    #     return 'Point Unreachable and stop'

    self.bb = math.sqrt(((self.d + self.a_0)**2)+((self.Pw_z-self.a_1)**2)) # backward facing

    # angles
    # forward facing solutions
    self.beta = math.acos((self.d - self.a_0)/self.b)
    self.alpha = math.acos(((self.b**2)+(self.a_2**2)-(self.c**2))/(2*self.b*self.a_2))

    # backward facing solutions
    self.delta = math.acos(((self.a_2**2)+(self.c**2)-(self.b**2))/(2*self.a_2*self.c))
    self.beta2 = math.acos((self.d + self.a_0)/self.bb)
    self.alpha2 = math.acos(((self.bb**2)+(self.a_2**2)-(self.c**2)) / (2*self.bb*self.a_2))

    # First 4 Solutions
    self.CalculateFfElbowUp()
    self.CalculateFfElbowDn()
    self.CalculateBfElbowUp()
    self.CalculateBfElbowDn()

    #
    self.Solutions.bfElbowUpN    = copy(self.Solutions.bfElbowUpP)
    self.Solutions.bfElbowUpN[0] = self.Solutions.ffElbowUp[0] + 180

    #
    self.Solutions.bfElbowDnN    = copy(self.Solutions.bfElbowDnP)
    self.Solutions.bfElbowDnN[0] = self.Solutions.ffElbowDn[0] + 180


  def CalculateFfElbowUp(self):

    if self.Pw_x == 0 and self.Pw_y == 0:
        print("Shoulder Singularity Detected")
        theta_1 = 0  # assign any value make last value in lin
        # th1 = None # th1 is undefined at singularity
    else:
        # calculate theta_1
        theta_1 = -np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))
    theta_1_1 = theta_1 - 360
    theta_1_2 = theta_1 + 360

    # calculate theta_2
    theta_2 = -math.degrees(self.alpha + self.beta)

    # calculate theta_3
    self.m = math.acos(((self.c**2)+(self.b**2)-(self.a_2**2)) / (2*self.b*self.c))
    self.s = 180 - self.alpha - (self.m + self.q) #for triangle Pw S A3
    theta_3 = math.degrees(180 - self.s)

    # save the solution
    self.Solutions.ffElbowUp = [theta_1, theta_2, theta_3]
    self.Solutions.ffElbowUp_1 = [theta_1_1, theta_2, theta_3]
    self.Solutions.ffElbowUp_2 = [theta_1_2, theta_2, theta_3]


  def CalculateFfElbowDn(self):

    if self.Pw_x == 0 and self.Pw_y == 0:
        print("Shoulder Singularity Detected")
        theta_1 = 0  # assign any value make last value in lin
        # th1 = None # th1 isundefined at singularity
    else:
        # calculate theta_1
        theta_1 = -np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))
    theta_1_1 = theta_1 - 360
    theta_1_2 = theta_1 + 360

    # calculate theta_2
    theta_2 = -math.degrees(self.beta - self.alpha)

    # calculate theta_3
    self.w = math.acos(((self.c**2)+(self.b**2)-(self.a_2**2)) / (2*self.b*self.c))
    self.e = 180-(self.w-self.q)-self.alpha
    theta_3 = 180 - self.e
    theta_3 = math.degrees(-theta_3)

    # save the solution
    self.Solutions.ffElbowDn = [theta_1, theta_2, theta_3]
    self.Solutions.ffElbowDn_1 = [theta_1_1, theta_2, theta_3]
    self.Solutions.ffElbowDn_2 = [theta_1_2, theta_2, theta_3]

  def CalculateBfElbowUp(self):

    if self.Pw_x == 0 and self.Pw_y == 0:
        print("Shoulder Singularity Detected")
        theta_1 = 0  # assign any value make last value in lin
        # th1 = None # th1 isundefined at singularity
    else:
        # calculate theta_1
        theta_1   = -(np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))) - 180

    # calculate theta_2
    theta_2 = self.beta2 + self.alpha2
    theta_2 = - math.degrees(math.pi - theta_2)

    # calculate theta_3
    self.h = math.acos(((self.c**2)+(self.bb**2)-(self.a_2**2)) / (2 * self.c * self.bb))- self.q
    self.i = 180 - math.degrees(self.h + self.alpha2)
    theta_3 = -(180 - self.i)

    # save the solution
    self.Solutions.bfElbowUpP = [theta_1, theta_2, theta_3]

  def CalculateBfElbowDn(self):

    if self.Pw_x == 0 and self.Pw_y == 0:
        print("Shoulder Singularity Detected")
        theta_1 = 0  # assign any value make last value in lin
        # th1 = None # th1 isundefined at singularity
    else:
        # calculate theta_1
        theta_1   = -(np.degrees((np.arctan2(self.Pw_y, self.Pw_x)))) - 180

    # calculate theta_2
    theta_2 = self.beta2 - self.alpha2
    theta_2 = - (180 - math.degrees(theta_2))

    # calculate theta_3
    self.x = math.acos(((self.a_2**2)+(self.c**2)-(self.bb**2))/(2*self.a_2*self.c))
    self.y = 180 - math.degrees(self.x)
    self.z = 180 - math.degrees(self.q) - self.y
    theta_3 = 180 - self.z

    # save the solution
    self.Solutions.bfElbowDnP = [theta_1, theta_2, theta_3]

  def CalculateWCP(self):
    self.a_e  = [self.TCP[0][2], self.TCP[1][2], self.TCP[2][2]] # a_e is the last column of the rotation matrix
    self.Pw_x = (self.TCP[0][3] - self.d6 * self.a_e[0])
    self.Pw_y = (self.TCP[1][3] - self.d6 * self.a_e[1])
    self.Pw_z = (self.TCP[2][3] - self.d6 * self.a_e[2])
    # print(self.Pw_x,self.Pw_y,np.degrees(-np.arctan2(0,-1550))) #why its not 0?

class RobotArm:

    def compute_dh_matrix(self, a, alpha, d, theta):
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]])

    def forward_kinematics(self, inp):
        dh_parameters = [
            [0, 180, 0, 0],
            [330, 90, -645, inp[0]],
            [1150, 0, 0, inp[1]],
            [115, 90, 0, inp[2]-90],
        ]

        matrices = [self.compute_dh_matrix(*params) for params in dh_parameters]
        result_matrix = np.eye(4)

        for matrix in matrices:
            result_matrix = np.dot(result_matrix, matrix)

        return result_matrix  # Extract the translation values