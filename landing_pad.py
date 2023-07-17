import pygame

from constants import (
    LANDING_PAD_WIDTH,
    LANDING_PAD_HEIGHT,
    LANDING_PAD_COLOR,
    HEIGHT,
    ROCKET_HEIGHT,
)


class LandingPad:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.width = 30
        self.height = 10

    def draw(self, screen):
        pygame.draw.ellipse(
            screen,
            LANDING_PAD_COLOR,
            pygame.Rect(
                self.x - LANDING_PAD_WIDTH / 2,
                HEIGHT - (self.y - ROCKET_HEIGHT / 2 + LANDING_PAD_HEIGHT),
                LANDING_PAD_WIDTH,
                LANDING_PAD_HEIGHT,
            ),
        )

    def update(self):
        pass
