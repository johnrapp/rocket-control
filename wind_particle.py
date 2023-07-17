import random

import pygame

from constants import (
    WIDTH,
    HEIGHT,
    WIND_COLOR,
    WIND_PARTICLE_THICKNESS,
    WIND_PARTICLE_LENGTH,
)


class WindParticle:
    def __init__(self, direction_x, direction_y):
        self.spawn()

        self.direction_x = direction_x
        self.direction_y = direction_y

    def update(self):
        self.x = (self.x + self.direction_x + 50) % (WIDTH + 100) - 50
        self.y = (self.y + self.direction_y + 50) % (HEIGHT + 100) - 50

    def spawn(self):
        self.x = random.uniform(0, WIDTH)
        self.y = random.uniform(0, HEIGHT)

    def draw(self, screen):
        pygame.draw.line(
            screen,
            WIND_COLOR,
            (self.x, HEIGHT - self.y),
            (
                self.x + self.direction_x * WIND_PARTICLE_LENGTH,
                HEIGHT - (self.y + self.direction_y * WIND_PARTICLE_LENGTH),
            ),
            WIND_PARTICLE_THICKNESS,
        )
