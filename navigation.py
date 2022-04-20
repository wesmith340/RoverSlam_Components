from controls import NavType, ControlType
from path import Coord, Slam
from ObjectDetection.MRO import hell_yeah as ObjDetection
# import objDetection as objDet



navType = NavType.FIND_QR
foundDigsite = False
prevControl = ControlType.NOTHING
slam = Slam()

def navLoop():
    control = ControlType.NOTHING
    if navType == NavType.FIND_QR:
        control = findQR()
    elif navType == NavType.INIT_DIG:
        control = initDig()
    elif navType == NavType.FOLLOW_PATH:
        control = followPath()
    elif navType == ControlType.DIGGING:
        control = dig()
    elif navType == ControlType.DUMPING:
        control = dump()
    
    return control

def findQR():
    # TODO
    # Implement find QR code procedures

    # if QR code found and oriented correctly
    if foundDigsite:
        navType = NavType.FOLLOW_PATH
    else: 
        navType = NavType.INIT_DIG

def initDig():
    global navType, foundDigsite
    slam.saveHome()
    navType = NavType.FOLLOW_PATH
    foundDigsite = True
    return ControlType.STOP
     

def followPath():
    if ObjDetection.control != ControlType.NOTHING:
        return ObjDetection.control
    else:
        return slam.control

def dump():
    pass

def dig():
    pass

if __name__ == '__main__':
    while True:
        control = navLoop()
        if control != prevControl:
            # TODO
            # push new control to EE team
            print(control)
            prevControl = control