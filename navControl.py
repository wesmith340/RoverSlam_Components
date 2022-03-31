from .controls import NavType, ControlType
import slam
# import objDetection as objDet



navType = NavType.FIND_QR
foundDigsite = False

prevControl = ControlType.NOTHING

def navLoop():
    control = ControlType.NOTHING
    if navType == NavType.FIND_QR:
        control = findQR()
    elif navType == NavType.INIT_DIG:
        control = initDig()
    elif navType == NavType.FOLLOW_PATH:
        control = followPath()
    elif navType == NavType.DIGGING:
        control = dig()
    elif navType == NavType.DUMPING:
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
    control = slam.control
    pass

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
            prevControl = control