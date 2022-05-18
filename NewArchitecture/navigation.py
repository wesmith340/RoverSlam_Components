from packages.Pathfinding.pathfinding import slam
from packages.Pathfinding.qrfinding import qrFinder
from packages.Serial_Commands import controller
from packages.Serial_Commands.controls import ControlType, Precedence

from packages.ObjectDetection.MRO import hell_yeah

import threading
import time

SENDER_TYPE = Precedence.NAVIGATION
DIG_TIME = 1000
DUMP_TIME = 1000

class Waiter(threading.Condition):
    def wait(self):
        self.acquire()
        super().wait()
        self.release()
    
    def notify_all(self) -> None:
        self.acquire()
        super().notify_all()
        self.release()

running = False
c = Waiter()
def navLoop():
    global running, c
    running = True

    if running:
        print('Finding QR')
        # findQR(c)
        # c.wait()
    time.sleep(2)
    slam.clear()
    if running:
        slam.rotate180(c)
        c.wait()

    if running:
        slam.saveHome()
        slam.startFollowing(c)
        print('Following path')
    while running:
        c.wait()
        controller.sendCommand(SENDER_TYPE, ControlType.DIGGING)
        print(SENDER_TYPE, ControlType.DIGGING)
        time.sleep(DIG_TIME)
        controller.sendCommand(SENDER_TYPE, ControlType.STOP)
        print(SENDER_TYPE, ControlType.STOP)
        controller.sendCommand(SENDER_TYPE, ControlType.NOTHING)

        if running:
            slam.flipPath(c)
            print('Following path')
            c.wait()

        if running:
            print('Finding QR')
            # findQR(c)
            # c.wait()
        if running:
            controller.sendCommand(SENDER_TYPE, ControlType.DUMPING)
            print(SENDER_TYPE, ControlType.DUMPING)
            time.sleep(DUMP_TIME)
            controller.sendCommand(SENDER_TYPE, ControlType.STOP)
            print(SENDER_TYPE, ControlType.STOP)
            print(SENDER_TYPE, ControlType.NOTHING)

        if running:
            slam.flipPath(c)

def simpleNavLoop():
    c = Waiter()
    slam.clear()

    slam.saveHome()
    slam.startFollowing(c)
    print('Following path')
    c.wait()

    controller.sendCommand(SENDER_TYPE, ControlType.DIGGING)
    print(SENDER_TYPE, ControlType.DIGGING)
    time.sleep(DIG_TIME)
    controller.sendCommand(SENDER_TYPE, ControlType.STOP)
    print(SENDER_TYPE, ControlType.STOP)

    slam.flipPath(c)
    print('Following path')
    c.wait()


def findQR(c: threading.Condition):
    c.notify_all()
    # TODO
    pass

keys = {
    'w':ControlType.FORWARD,
    's':ControlType.BACKWARD,
    'a':ControlType.LEFT,
    'd':ControlType.RIGHT,
    '':ControlType.STOP,
    'i':ControlType.DIGGING,
    'o':ControlType.DUMPING,
}

def manual():
    print('wasd movement\ni:dig\no:dump\nq:exit manual control')
    inManual = True
    while inManual:
        usrIn = input()
        if usrIn in keys:
            controller.sendCommand(Precedence.MANUAL, keys[usrIn])
            print(keys[usrIn])
        elif usrIn == 'q':
            controller.sendCommand(Precedence.MANUAL, ControlType.NOTHING)
            inManual = False
fullAutoThread = threading.Thread(target=navLoop)
partAutoThread = threading.Thread(target=simpleNavLoop)

if __name__ == '__main__':

    print('1:Full Auto\n2:Partial Auto\n3:Manual\nq:exit program')
    usrIn = input()
    while usrIn != 'q':
        if usrIn == '1':
            fullAutoThread.start()
        elif usrIn == '2':
            partAutoThread.start()
        elif usrIn == '3':
            running = False
            slam.clear()
            c.notify_all()
            manual()
        print('1:Full Auto\n2:Partial Auto\n3:Manual\nq:exit program')
        usrIn = input()
    running = False
