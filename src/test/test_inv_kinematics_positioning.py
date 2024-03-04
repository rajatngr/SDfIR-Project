from ctrl.kinematics.inverse.inv_kinematics_positioning import IKPosSolver


# TODO: add test cases
class TestInvKinPos:
    def test_solve(self):
        input_matrix_A = [[-1, 0, 0, -1550],
                          [0, -1, 0, 0],
                          [0, 0, 1, 2125],
                          [0, 0, 0, 1]]

        input_matrix_B = [[-8.85562315e-01, 2.69213051e-02, -4.63739830e-01, 9.16922244e+02],
                          [-4.47193236e-01, -3.19530365e-01, 8.35415199e-01, 1.56699319e+02],
                          [-1.25688490e-01, 9.47193533e-01, 2.95003075e-01, 2.66453955e+03],
                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

        solver = IKPosSolver(input_matrix_B)

        solutions = solver.Solutions.allSolutions()

        print("\n")
        [print(sol) for sol in solutions]
