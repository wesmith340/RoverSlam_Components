import serial, time
from controls import ControlType
from serial.tools.list_ports import comports

for c in comports():
    print(c)

def sendCommand(port, command):
    port.write(command)


arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600)

# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value


# arduino = serial.Serial('COM1', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

# Variable to keep our game loop running
running = True
  

keys = {
    'w':ControlType.FORWARD,
    's':ControlType.BACKWARD,
    'a':ControlType.LEFT,
    'd':ControlType.RIGHT,
    '':ControlType.STOP
}

while running:
    usrIn = input()
    if usrIn in keys:
        arduino.write(f'{keys[usrIn].value}'.encode())
        print(keys[usrIn])
        print(arduino.readline().decode())
