from packages.Pathfinding.pathfinding import slam
from packages.Pathfinding.qrfinding import qrFinder
# from .Serial_Commands import controller
from packages.Serial_Commands.controls import ControlType, Precedence

from threading import Condition
import time

SENDER_TYPE = Precedence.NAVIGATION
DIG_TIME = 1000
DUMP_TIME = 1000


class Waiter(Condition):
    def wait(self):
        self.acquire()
        super().wait()
        self.release()
    
    def notify_all(self) -> None:
        self.acquire()
        super().notify_all()
        self.release()
    

def navLoop():
    c = Waiter()
    print('Finding QR')
    # findQR(c)
    # c.wait()
    time.sleep(10)
    slam.rotate180(c)
    # c.acquire()
    c.wait()
    # c.release()

    slam.saveHome()
    slam.startFollowing(c)
    print('Following path')
    while True:
        
        c.wait()
        # controller.sendCommand(SENDER_TYPE, ControlType.DIGGING)
        print(SENDER_TYPE, ControlType.DIGGING)
        time.sleep(DIG_TIME)
        # controller.sendCommand(SENDER_TYPE, ControlType.STOP)
        print(SENDER_TYPE, ControlType.STOP)

        slam.flipPath(c)
        print('Following path')
        c.wait()
        print('Finding QR')
        # findQR(c)
        # c.wait()
        # controller.sendCommand(SENDER_TYPE, ControlType.DUMPING)
        print(SENDER_TYPE, ControlType.DUMPING)
        time.sleep(DUMP_TIME)
        # controller.sendCommand(SENDER_TYPE, ControlType.STOP)
        print(SENDER_TYPE, ControlType.STOP)
        slam.flipPath(c)


def findQR(c: Condition):
    c.notify_all()
    # TODO
    pass


if __name__ == '__main__':
    navLoop()