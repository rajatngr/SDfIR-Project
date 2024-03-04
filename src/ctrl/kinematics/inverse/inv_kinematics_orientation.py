import numpy as np

from .inv_kinematics_positioning import IKPosSolver
from ...postypes.configuration import configuration


# TODO: add Type hints
class IKOriSolver:
    def __init__(self, TCP=None):

        # initialize
        self.R = None
        self.pos_solver = IKPosSolver()
        self.valid_positioning_solutions = None
        self.configurations = []

        # check if TCP was passed; if so: solve already
        if TCP is not None:
            self.TCP = TCP
            self.solve()

        # if we don't have a TCP, initialize to None and dont solve yet
        else:
            self.TCP = None

    def solve(self, TCP=None):

        # if we get a (new) TCP, update our variables
        if TCP is not None:
            self.TCP = TCP

        # otherwise, check if it was initialized in the constructor
        else:
            assert self.TCP is not None

        self.R = np.array([row[:3] for row in self.TCP[:3]])  # rotation matrix
        self.pos_solver.solve(self.TCP)
        self.valid_positioning_solutions = self.pos_solver.Solutions.validSolutions()
        print(self.valid_positioning_solutions)
        self.calculate_valid_solutions()

    def calculate_valid_solutions(self):
        # TODO: comments, structure, magic numbers (should be robot constants, new class?), naming, DRY, ...
        for sol in self.valid_positioning_solutions:
            fkmatrix = self.forward_kinematics(sol)

            # Extract the first 3x3 submatrix
            R0_3 = fkmatrix[:3, :3]

            R3_6 = np.dot(np.transpose(R0_3), self.R)

            theta_5_P = np.degrees(np.arccos(-R3_6[2][2]))
            theta_5_N = -theta_5_P

            #TODO: is this correct? add test case #dont have a test cases for singularities , Could you plz provide some?
            #Check wrist singularity
            if theta_5_P == 0:
                if np.isclose(R3_6[2][2], -1):
                    # Wrist singularity detected
                    theta_46_sum = np.degrees(np.arctan2(R3_6[1][0], R3_6[0][0]))
                    theta_4 = 0   #how many combinations will we get for wrist singularity case?
                    theta_6 = theta_46_sum - theta_4
                    print("Wrist Singularity Detected:")
                    self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4, theta_5_P, theta_6]))
            else:
            #Calculate theta_4 from R3_6 elements
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

                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_III, theta_5_P, theta_6_I]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_III, theta_5_P, theta_6_III]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_I, theta_5_P, theta_6_I]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_I, theta_5_P, theta_6_III]))

                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_II, theta_5_N, theta_6_IV]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_II, theta_5_N, theta_6_II]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_IV, theta_5_N, theta_6_IV]))
                self.configurations.append(configuration([sol[0], sol[1], sol[2], theta_4_IV, theta_5_N, theta_6_II]))

    def compute_dh_matrix(self, a, alpha, d, theta):
        # TODO: make these non-static
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]])

    def forward_kinematics(self, inp):
        # TODO: make these non-static
        dh_parameters = [
            [0, 180, 0, 0],
            [330, 90, -645, inp[0]],
            [1150, 0, 0, inp[1]],
            [115, 90, 0, inp[2] - 90],
        ]

        matrices = [self.compute_dh_matrix(*params) for params in dh_parameters]
        result_matrix = np.eye(4)

        for matrix in matrices:
            result_matrix = np.dot(result_matrix, matrix)

        return result_matrix  # Extract the translation values