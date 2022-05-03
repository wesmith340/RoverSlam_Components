from enum import Enum

class NavType(Enum):
    FIND_QR = 1
    FOLLOW_PATH = 2
    INIT_DIG = 3
    NOTHING = -1
    
class ControlType(Enum):
    FORWARD = 0
    RIGHT = 1
    LEFT = 2
    STOP = 3
    BACKWARD = 4
    DIGGING = 5
    DUMPING = 6
    NOTHING = -1

class Precedence(Enum):
    NAVIGATION = 0
    OBJECT_DETECTION = 1
    QR_CODE = 2
    PATH_FINDING = 3
    NOTHING = 4
    