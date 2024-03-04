import os
import sys

# add parent dir to path
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)

from misc import jointRangeCheck


class InvKinPosSet:

    def __init__(self):
        # TODO: think about whether we actually need to keep track of which solution is which here
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

    def allSolutions(self):
        sols = [self.ffElbowUp,  self.ffElbowDn,  self.bfElbowUpP,  self.bfElbowDnP,
                self.bfElbowUpN, self.bfElbowDnN, self.ffElbowUp_1, self.ffElbowUp_2, self.ffElbowDn_1, self.ffElbowDn_2]

        return [sol for sol in sols if sol is not None]

    def validSolutions(self):
        return jointRangeCheck.check(self)


class FwKinPosSet:
    def __init__(self):
        pass


if __name__ == '__main__':
    pass