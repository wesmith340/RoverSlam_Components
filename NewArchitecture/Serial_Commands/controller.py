import serial, time
from controls import ControlType, Precedence
from serial.tools.list_ports import comports
from threading import Lock


for c in comports():
    print(c)

def sendCommand(port, command):
    port.write(command)


arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600)

# arduino = serial.Serial('COM1', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

curSender = Precedence.NOTHING

def sendCommand(sender:Precedence, command:ControlType):
    Lock.acquire()
    global curSender, arduino
    if curSender.value <= sender.value:
        if command == ControlType.NOTHING:
            curSender = Precedence.NOTHING
        else:
            arduino.write(f'{command.value}'.encode())
            curSender = sender
    Lock.release()
