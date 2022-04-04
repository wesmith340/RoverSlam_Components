import math
from random import randrange
import pygame
from typing import List, Tuple


class Vector2:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def getTuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def copy(self) -> object:
        return Vector2(self.x, self.y)

    def __add__(self, other: object) -> object:
        self.x += other.x
        self.y += other.y
        return self

    def __iadd__(self, other: object) -> object:
        self.x += other.x
        self.y += other.y
        return self

    def __mul__(self, scalar: float) -> object:
        self.x *= scalar
        self.y *= scalar
        return self

    def __imul__(self, scalar: float) -> object:
        return self * scalar


pygame.init()
SCALE = 0.5
WIDTH = 900
HEIGHT = 900
ORIGIN = Vector2(WIDTH, HEIGHT)
screen = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
pygame.display.set_caption("Point2Point Navigation")


class Rover:
    def __init__(self, rotation=math.pi, position: Vector2 = Vector2(0, 0)) -> None:
        self.rotation = rotation
        self.position: Vector2 = position
        self.SIZE = 30
        self.ROTATIONSTEP = math.pi/1024
        self.STEP = 0.1
        self.path: List[Vector2] = []

    def addToPath(self, point: Vector2) -> None:
        self.path.append(point)

    def setRotation(self, rotation: float) -> None:
        self.rotation = rotation

    def setPosition(self, position: Vector2) -> None:
        self.position = position

    def draw(self, screen) -> None:
        # draw path history
        for i, point in enumerate(self.path):
            point = point.copy() + ORIGIN
            point *= SCALE
            if i != 0:
                lastPoint: Vector2 = self.path[i-1].copy() + ORIGIN
                lastPoint *= SCALE
                pygame.draw.line(screen, (255, 0, 0),
                                 lastPoint.getTuple(), point.getTuple())
            pygame.draw.circle(screen, (0, 255, 0),
                               point.getTuple(), 10 * SCALE)

        # draw rover and rotation
        rover: Vector2 = self.position.copy() + ORIGIN
        rover *= SCALE
        rotVec: Vector2 = Vector2(
            math.sin(self.rotation), math.cos(self.rotation))
        rotVec *= self.SIZE
        rotVec *= SCALE
        line: Vector2 = rover.copy() + rotVec

        pygame.draw.circle(screen, (0, 0, 255), rover.getTuple(), self.SIZE * SCALE)
        pygame.draw.line(screen,
                         (255, 0, 255),
                         rover.getTuple(),
                         line.getTuple())

    def turn(self, direction: str) -> None:
        if direction == 'LEFT':
            self.rotation += self.ROTATIONSTEP
        elif direction == 'RIGHT':
            self.rotation -= self.ROTATIONSTEP

    def move(self, direction: str) -> None:
        move = Vector2(math.sin(self.rotation), math.cos(self.rotation))
        move *= self.STEP
        if direction == 'BACKWARD':
            move *= -1
        self.position += move


ROVER = Rover(0)

done = False
while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    screen.fill((255, 255, 255))
    ROVER.draw(screen)
    pygame.display.flip()

pygame.quit()
