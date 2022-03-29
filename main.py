import math
from random import randrange
import pygame


class Rover:
    def __init__(self, rotation=math.pi, position=(0, 0)) -> None:
        self.rotation = rotation
        self.x = position[0]
        self.y = position[1]
        self.SIZE = 30
        self.ROTATIONSTEP = math.pi/1024
        self.STEP = 0.01

    def draw(self, screen) -> None:
        pygame.draw.circle(screen, (0, 0, 255), (self.x, self.y), self.SIZE)
        pygame.draw.line(screen,
                         (255, 0, 255),
                         (self.x, self.y),
                         (self.x + math.sin(self.rotation) * self.SIZE,
                          self.y + math.cos(self.rotation) * self.SIZE))

    def turn(self, direction: str) -> None:
        if direction == 'LEFT':
            self.rotation += self.ROTATIONSTEP
        elif direction == 'RIGHT':
            self.rotation -= self.ROTATIONSTEP

    def move(self, direction: str) -> None:
        if direction == 'FORWARD':
            self.x += math.sin(self.rotation) * self.STEP
            self.y += math.cos(self.rotation) * self.STEP
        elif direction == 'BACKWARD':
            self.x -= math.sin(self.rotation) * self.STEP
            self.y -= math.cos(self.rotation) * self.STEP
    
    def navigate():
        pass


class Path:
    def __init__(self, width: int, height: int, resolution: int) -> None:
        self.path = [(width/2, 0)]
        for i in range(100, height, resolution):
            self.path.append((randrange(0, width), i))
        self.path.append((width/2, height))

    def draw(self, screen) -> None:
        for i, point in enumerate(self.path):
            if i != 0:
                pygame.draw.line(screen, (255, 0, 0),
                                 self.path[i-1], point)
            pygame.draw.circle(screen, (0, 255, 0), point, 10)


pygame.init()
WIDTH, HEIGHT = 400, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
pygame.display.set_caption("Point2Point Navigation")

PATH = Path(WIDTH, HEIGHT, 50)
ROVER = Rover(0, PATH.path[2])

done = False
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    screen.fill((255, 255, 255))
    PATH.draw(screen)
    ROVER.draw(screen)
    ROVER.turn('RIGHT')
    ROVER.move('FORWARD')
    pygame.display.flip()

pygame.quit()
