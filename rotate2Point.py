import math
from typing import Tuple

def rotate2Point(currRot : float, currPos : Tuple(float, float), destination : Tuple(float, float)) -> str:
    pass

assert rotate2Point(0, (0, 1), (0, -1)) == 'LEFT'
assert rotate2Point(math.pi, (0, 1), (0, -1)) == 'LEFT'
assert rotate2Point(-math.pi, (0, 1), (0, -1)) == 'RIGHT'