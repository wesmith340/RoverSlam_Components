import serial, time
import pygame
from controls import ControlType
from serial.tools.list_ports import comports

for c in comports():
    print(c)

def sendCommand(port, command):
    port.write(command)


arduino = serial.Serial(port='COM11', baudrate=9600)

# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value


# arduino = serial.Serial('COM1', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle


# Define the background colour
# using RGB color coding.
background_colour = (234, 212, 252)
  
# Define the dimensions of
# screen object(width,height)
screen = pygame.display.set_mode((300, 300))

# Set the caption of the screen
pygame.display.set_caption('Manual Control')
  
# Fill the background colour to the screen
screen.fill(background_colour)

# Update the display using flip
pygame.display.flip()
  
# Variable to keep our game loop running
running = True
  
class InputKey:
    def __init__(self, moveType):
        self.pressed = False
        self.moveType = moveType

keys = {
    pygame.K_w:InputKey(ControlType.FORWARD),
    pygame.K_s:InputKey(ControlType.BACKWARD),
    pygame.K_a:InputKey(ControlType.LEFT),
    pygame.K_d:InputKey(ControlType.RIGHT)
}

prevC = ControlType.STOP

# game loop
while running:
# for loop through the event queue  
    keyPress = False
    for event in pygame.event.get():
      
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key in keys:
                keys[event.key].pressed = True
        elif event.type == pygame.KEYUP:
            if event.key in keys:
                keys[event.key].pressed = False

    c = ControlType.STOP

    for k in keys:
        if keys[k].pressed:
            c = keys[k].moveType

    if c != prevC:
        arduino.write(f'{c.value}'.encode())
        print(c)
        
    prevC = c
