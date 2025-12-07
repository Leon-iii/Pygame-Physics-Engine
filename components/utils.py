from components.vector import Vector2D
from typing import Optional
import pygame

def cartesian_coord_from_pygame(
    pygame_coordinates: Vector2D,
    surface: Optional["pygame.Surface"] = None,
) -> Vector2D:
    # surface 인자를 안 넘기면 현재 display surface를 자동으로 사용
    if surface is None:
        surface = pygame.display.get_surface()
        if surface is None:
            raise RuntimeError("No active display Surface. Call pygame.display.set_mode(...) first.")

    # 높이 기준으로 y 반전
    width, height = surface.get_size()
    return Vector2D(pygame_coordinates[0], height - pygame_coordinates[1])

def pygame_coord_from_cartesian(
    cartesian_coordinates: Vector2D,
    surface: Optional["pygame.Surface"] = None,
) -> tuple[int, int]:
    # surface 인자를 안 넘기면 현재 display surface를 자동으로 사용
    if surface is None:
        surface = pygame.display.get_surface()
        if surface is None:
            raise RuntimeError("No active display Surface. Call pygame.display.set_mode(...) first.")

    # 높이 기준으로 y 반전
    width, height = surface.get_size()
    return Vector2D(cartesian_coordinates[0], height - cartesian_coordinates[1])

# def pygame_coord_from_cartesian(cartesian_coordinates: Vector2D, surface: "pygame.Surface") -> tuple[float, float]:

#     return (cartesian_coordinates[0], surface.get_height() - cartesian_coordinates[1])