from .Pathfinding.pathfinding import slam
from .Pathfinding.qrfinding import qrFinder
from .Serial_Commands import controller
from .Serial_Commands.controls import ControlType, Precedence

from threading import Condition
import time

SENDER_TYPE = Precedence.NAVIGATION
DIG_TIME = 1000
DUMP_TIME = 1000


def navLoop():
    c = Condition()
    qrFinder.findQR(c)
    c.wait()
    slam.rotate180(c)
    c.wait()

    slam.saveHome()

    while True:
        slam.startFollowing(c)
        c.wait()

        controller.sendCommand(SENDER_TYPE, ControlType.DIGGING)
        time.sleep(DIG_TIME)
        controller.sendCommand(SENDER_TYPE, ControlType.STOP)

        slam.startFollowing(c)
        c.wait()
        findQR(c)
        c.wait()

        controller.sendCommand(SENDER_TYPE, ControlType.DUMPING)
        time.sleep(DUMP_TIME)
        controller.sendCommand(SENDER_TYPE, ControlType.STOP)


def findQR(c: Condition):
    # TODO
    pass


if __name__ == '__main__':
    navLoop()