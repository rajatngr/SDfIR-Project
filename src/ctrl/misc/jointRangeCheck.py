import os
import sys

# add parent dir to path
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)


# expects an InvKinPosSet object
def check(solutionSet):
    # TODO: add documentation
    # TODO: think about whether we need to keep track of which solution is which, because we're not atm
    valid_solutions = []

    for solution in solutionSet.allSolutions():
        if -185 < solution[0] < 185 and -140 < solution[1] < -5 and -120 < solution[2] < 168:
            valid_solutions.append(solution)

    return valid_solutions


if __name__ == '__main__':
    pass
