from ...postypes.SixDPos import SixDPos
from ...postypes.configuration import configuration
import math
import numpy as np


class FwKinematics:
    def get_fw_kinematics(self, config: configuration) -> SixDPos:
        # TODO: Implement the direct kinematics
        # Implement your logic here to compute the forward kinematics and derive position and euler angles.
        # Consider your definitions of rotations (degrees or radians) and units (meters or centimeters).
        # Modify the return statement accordingly.

        # Example usage:
        # inp = [20, -80, 100, -45, 50, -10]  # Your input list
        inp = config.get_configuration()
        print(inp)

        inp = [math.degrees(x) for x in inp]

        end_effector = self.forward_kinematics(inp)
        print(end_effector)

        R = np.array([row[:3] for row in end_effector[:3]])
        print(R)

        # Extract individual elements from the rotation matrix for convenience
        r11, r12, r13 = R[0, :]
        r21, r22, r23 = R[1, :]
        r31, r32, r33 = R[2, :]

        # Check for Gimbal lock singularity
        if r11 == 0 and r21 == 0 and r32 == 0 and r33 == 0:
            print("Gimbal Lock Detected")
            yaw = np.arcsin(-r12)
            pitch = -(r31 * np.pi) / 2
            roll = 0
        else:
            # Calculate Euler angles normally from the matrix and co-ordinates
            yaw     = np.arctan2(end_effector[1][0], end_effector[0][0])
            pitch   = np.arctan2(-end_effector[2][0], np.sqrt(end_effector[2][1] ** 2 + end_effector[2][2] ** 2))
            roll    = np.arctan2(end_effector[2][1], end_effector[2][2])

        # print 6 values for UI 3 position co-ordinates & 3 euler angles
        result = SixDPos(end_effector[0][3]/1000, end_effector[1][3]/1000, end_effector[2][3]/1000, yaw, pitch, roll)
        print(np.degrees(yaw), np.degrees(pitch), np.degrees(roll))
        print(result.get_position())
        # For demonstration purposes, a fixed SixDPos is returned. Replace this with your computation.
        return result

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
            [0, -90, -1220, inp[3]],
            [0, 90, 0, inp[4]],
            [0, 180, -215, inp[5]+180]
        ]

        matrices = [self.compute_dh_matrix(*params) for params in dh_parameters]
        result_matrix = np.eye(4)

        for matrix in matrices:
            result_matrix = np.dot(result_matrix, matrix)

        return result_matrix  # Extract the translation values