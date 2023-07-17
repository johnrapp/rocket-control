import pygame
import sys
import pandas as pd

from optimize_control_sequence import optimize_control_sequence

# Set up some constants
from constants import (
    BACKGROUND_COLOR,
    WIDTH,
    HEIGHT,
    ROCKET_HEIGHT,
    FPS,
    N_WIND_PARTICLES,
    BACKGROUND_IMAGE,
)

from rocket import Rocket
from wind_particle import WindParticle
from landing_pad import LandingPad

scenario = {
    "time_steps": 80,
    "solve_limit": 15,
    "wind_direction_x": 4,
    "wind_direction_y": 0,
    "x0": WIDTH / 2,
    "y0": HEIGHT / 2,
    "x_target": WIDTH / 2 - 100,
    "y_target": ROCKET_HEIGHT / 2,
    "velocity_x0": 0,
    "velocity_y0": 0,
    "rotation0": 0,
    "rotational_velocity0": 0,
}


def main():
    pygame.init()
    # screen = pygame.display.set_mode((WIDTH, HEIGHT))
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.SRCALPHA)

    # model_output = optimize_control_sequence(scenario)
    # model_output = pd.DataFrame()

    # rocket = Rocket(x0, y0, wind_direction_x, wind_direction_y, model_output)
    rocket = Rocket(
        scenario,
        # model_output=model_output,
        mode="playback",
    )
    landing_pad = LandingPad(scenario["x_target"], scenario["y_target"])

    wind_particles = []
    for _ in range(N_WIND_PARTICLES):
        wind_particles.append(
            WindParticle(scenario["wind_direction_x"], scenario["wind_direction_y"])
        )

    entities = wind_particles + [landing_pad]

    draw(screen, rocket, entities)
    reoptimize(screen, rocket)

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        update(screen, rocket, entities)

        draw(screen, rocket, entities)

        clock.tick(FPS)


def update(screen, player, entities):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_SPACE]:
        reoptimize(screen, player)

    player.update()
    for e in entities:
        e.update()


def reoptimize(screen, rocket):
    new_scenario = scenario.copy()

    new_scenario.update(
        {
            "x0": rocket.x,
            "y0": rocket.y,
            "velocity_x0": rocket.velocity_x,
            "velocity_y0": rocket.velocity_y,
            "rotation0": rocket.rotation,
            "rotational_velocity0": rocket.rotational_velocity,
        }
    )

    print(new_scenario)

    draw_optimizing(screen)

    for time_steps in [50, 80, 110, 150, 200]:
        new_scenario["time_steps"] = time_steps
        new_scenario["solve_limit"] = (time_steps / 100) * 8 + 5
        model_output = optimize_control_sequence(new_scenario)
        if model_output is not None:
            rocket.reroute(model_output)
            break

def draw_optimizing(screen):
    font = pygame.font.Font(None, 60)
    text = font.render("Taking the wheel...", True, (255, 255, 255))

    text_rect = text.get_rect(center=(WIDTH // 2, 200))

    screen.blit(text, text_rect)
    pygame.display.flip()


background_image = pygame.transform.scale(BACKGROUND_IMAGE, (WIDTH, HEIGHT))


def draw(screen, player, entities):
    screen.blit(background_image, (0, 0))

    for e in entities:
        e.draw(screen)

    player.draw(screen)

    pygame.display.flip()


if __name__ == "__main__":
    main()
