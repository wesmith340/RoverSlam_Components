import serial, time
from .controls import ControlType, Precedence
from serial.tools.list_ports import comports
from threading import RLock

ports = comports()
for c in ports:
    print(c)

arduino = serial.Serial(port=ports[0].device, baudrate=9600)

# arduino = serial.Serial('COM1', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

curSender = Precedence.NOTHING
lock = RLock()
def sendCommand(sender:Precedence, command:ControlType):
    lock.acquire()
    global curSender, arduino
    if curSender.value >= sender.value:
        if command == ControlType.NOTHING:
            curSender = Precedence.NOTHING
        else:
            print('sending info')
            arduino.write(f'{command.value}'.encode())
            curSender = sender
            
    lock.release()
