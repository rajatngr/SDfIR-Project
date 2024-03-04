from ctrl.kinematics.inverse.inv_kinematics import InvKinematics
from ctrl.postypes import configuration
from ctrl.postypes.SixDPos import SixDPos


class TestInvKin:
    # TODO: add tests
    def test_get_inv_kinematics(self):

        # test with an input matrix
        print("\nTESTING USING INPUT MATRIX --------------------------------")
        input_matrix_A = [[-1, 0, 0, -1550],
                          [0, -1, 0, 0],
                          [0, 0, 1, 2125],
                          [0, 0, 0, 1]]
        InvKinObj1 = InvKinematics()
        configuration_list = InvKinObj1.get_inv_kinematics(None, input_matrix_A)
        [print("p(", config.get_configuration(), ")") for config in configuration_list]

        # test with an input pose
        print("TESTING USING 6DPOS --------------------------------------")
        InvKinObj2 = InvKinematics()
        pose = SixDPos([-1500, 0, 500, 0.4, -1.5, 0.6])
        configuration_list = InvKinObj2.get_inv_kinematics(pose)
        [print("p(", config.get_configuration(), ")") for config in configuration_list]

    def test_six_dpos_2_tmat(self):
        assert False