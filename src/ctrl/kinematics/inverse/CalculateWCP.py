def calculateWCP(TCP, d6):
    # TODO: add documentation
    a_e = [TCP[0][2], TCP[1][2], TCP[2][2]]  # a_e is the last column of the rotation matrix
    Pw_x = (TCP[0][3] - d6 * a_e[0])
    Pw_y = (TCP[1][3] - d6 * a_e[1])
    Pw_z = (TCP[2][3] - d6 * a_e[2])
    return [Pw_x, Pw_y, Pw_z]