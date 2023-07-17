import pygame
import numpy as np
import pandas as pd

# Set up some constants
from constants import (
    ROCKET_COLOR,
    ROCKET_COLOR_ROBOT,
    FLAME_COLOR,
    SIDE_THRUSTER_COLOR,
    WIDTH,
    HEIGHT,
    ROCKET_WIDTH,
    ROCKET_HEIGHT,
    BOTTOM_THRUSTER_MAX_SIZE,
    SIDE_THRUSTER_MAX_SIZE,
    SIDE_THRUSTER_OFFSET,
    GRAVITY_FORCE,
    THRUST_FORCE,
    SIDE_THRUST_FORCE,
    DAMPING_FACTOR,
    AIR_RESISTANCE_FACTOR,
)


class Rocket:
    def __init__(
        self, scenario, mode="playback",
    ):  # playback
        self.x = scenario["x0"]
        self.y = scenario["y0"]
        self.velocity_x = scenario["velocity_x0"]
        self.velocity_y = scenario["velocity_y0"]

        self.rotation = scenario["rotation0"]
        self.rotational_velocity = scenario["rotational_velocity0"]

        self.wind_direction_x = scenario["wind_direction_x"]
        self.wind_direction_y = scenario["wind_direction_y"]

        self.tick = 0
        self.mode = mode

        self.thrust = 0
        self.side_thrust = 0

        self.model_output = pd.DataFrame()

    def update(self):
        if self.mode == "simulate" or self.tick >= self.model_output.shape[0]:
            self.simulation_step()

        elif self.mode == "playback" and self.tick < self.model_output.shape[0]:
            self.playback_step()

        self.set_thrust()

        self.tick += 1

    def simulation_step(self):
        if self.tick > 0:
            is_landed = self.y <= ROCKET_HEIGHT / 2
            self.rotation *= DAMPING_FACTOR

            thrust_component_x = np.sin(self.rotation) * THRUST_FORCE * self.thrust
            thrust_component_y = np.cos(self.rotation) * THRUST_FORCE * self.thrust

            gravity_component = GRAVITY_FORCE if self.y > 0 else 0

            relative_velocity_x = self.velocity_x - self.wind_direction_x
            relative_velocity_y = self.velocity_y - self.wind_direction_y

            air_force_x = (
                -relative_velocity_x
                * np.abs(np.cos(self.rotation))
                * AIR_RESISTANCE_FACTOR
            )
            air_force_y = (
                -relative_velocity_y
                * np.abs(np.sin(self.rotation))
                * AIR_RESISTANCE_FACTOR
            )

            self.velocity_x += thrust_component_x + (
                air_force_x if not is_landed else 0
            )
            self.velocity_y += thrust_component_y + (
                gravity_component + air_force_y if not is_landed else 0
            )

            self.y += self.velocity_y
            self.x += self.velocity_x

            self.rotational_velocity += self.side_thrust * SIDE_THRUST_FORCE
            self.rotation += self.rotational_velocity

        if self.y - ROCKET_HEIGHT / 2 <= 0:
            self.y = ROCKET_HEIGHT / 2
            self.velocity_x = 0
            self.angular_velocity = 0

        if self.y > HEIGHT:
            self.y = HEIGHT

        if self.x > WIDTH:
            self.x = WIDTH

        if self.x < 0:
            self.x = 0
            
    def playback_step(self):
        self.x = self.model_output.x.iloc[self.tick]
        self.y = self.model_output.y.iloc[self.tick]
        self.rotation = self.model_output.rotation.iloc[self.tick]
        self.velocity_x = self.model_output.velocity_x.iloc[self.tick]
        self.velocity_y = self.model_output.velocity_y.iloc[self.tick]
        self.rotational_velocity = self.model_output.rotational_velocity.iloc[self.tick]

    def set_thrust(self):
        keys = pygame.key.get_pressed()
        if self.tick < self.model_output.shape[0]:
            self.thrust = self.model_output.thrust.iloc[self.tick]
            self.side_thrust = self.model_output.side_thrust.iloc[self.tick]
        else:
            if keys[pygame.K_UP]:
                self.thrust = 1
            else:
                self.thrust = 0

            if keys[pygame.K_LEFT]:
                self.side_thrust = -1
            elif keys[pygame.K_RIGHT]:
                self.side_thrust = 1
            else:
                self.side_thrust = 0

    def reroute(self, model_output):
        self.model_output = model_output
        self.tick = 0

    def draw(self, screen):
        rocket_rect = pygame.Surface(
            (ROCKET_WIDTH, ROCKET_HEIGHT)
        )  # Create a surface for the rocket

        rocket_color = (
            ROCKET_COLOR
            if self.tick >= self.model_output.shape[0]
            else ROCKET_COLOR_ROBOT
        )
        rocket_rect.set_colorkey((0, 0, 0))  # Make black colors transparent

        pygame.draw.rect(
            rocket_rect, rocket_color, rocket_rect.get_rect()
        )  # Draw the rocket on the surface

        rotated_rocket_rect = pygame.transform.rotate(
            rocket_rect, np.degrees(-self.rotation)
        )

        # Calculate the new position of the rocket
        rect = rotated_rocket_rect.get_rect(center=(self.x, HEIGHT - self.y))

        screen.blit(rotated_rocket_rect, rect.topleft)

        # Draw the flame
        flame_x = self.x + np.sin((-self.rotation)) * (
            ROCKET_HEIGHT / 2 + self.thrust * BOTTOM_THRUSTER_MAX_SIZE
        )
        flame_y = HEIGHT - (
            self.y
            - np.cos((-self.rotation))
            * (ROCKET_HEIGHT / 2 + self.thrust * BOTTOM_THRUSTER_MAX_SIZE)
        )
        pygame.draw.circle(
            screen,
            FLAME_COLOR,
            (int(flame_x), int(flame_y)),
            int(self.thrust * BOTTOM_THRUSTER_MAX_SIZE),
        )

        if self.side_thrust < 0:
            left_thruster_position = rotate_point(
                (
                    self.x - ROCKET_WIDTH // 2,
                    HEIGHT - (self.y - ROCKET_HEIGHT // 2) - SIDE_THRUSTER_OFFSET,
                ),
                (self.x, HEIGHT - self.y),
                self.rotation + np.pi,
            )
            pygame.draw.circle(
                screen,
                SIDE_THRUSTER_COLOR,
                (int(left_thruster_position[0]), int(left_thruster_position[1])),
                int(abs(self.side_thrust) * SIDE_THRUSTER_MAX_SIZE),
            )
        if self.side_thrust > 0:
            right_thruster_position = rotate_point(
                (
                    self.x + ROCKET_WIDTH // 2,
                    HEIGHT - (self.y - ROCKET_HEIGHT // 2) - SIDE_THRUSTER_OFFSET,
                ),
                (self.x, HEIGHT - self.y),
                self.rotation + np.pi,
            )
            pygame.draw.circle(
                screen,
                SIDE_THRUSTER_COLOR,
                (int(right_thruster_position[0]), int(right_thruster_position[1])),
                int(abs(self.side_thrust) * SIDE_THRUSTER_MAX_SIZE),
            )


def rotate_point(point, origin, angle):
    ox, oy = origin
    px, py = point

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)

    return qx, qy
