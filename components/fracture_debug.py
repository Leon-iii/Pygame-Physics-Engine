# fracture_debug.py 같은 데 두면 깔끔
from components.vector import Vector2D
from components.utils import pygame_coord_from_cartesian
import pygame

DEBUG_SPLIT_LINES: list[tuple[Vector2D, Vector2D]] = []
# (contact_point p, normal n) 를 저장

def debug_draw_split_lines(surface: "pygame.Surface"):
    screen_h = surface.get_height()
    L = 1000  # 선 길이

    for p, n in DEBUG_SPLIT_LINES:
        n = n.normalize()
        # 분할선의 방향은 normal에 수직
        t = Vector2D(-n.y, n.x)

        a = p - t * L
        b = p + t * L

        ax, ay = pygame_coord_from_cartesian(a)
        bx, by = pygame_coord_from_cartesian(b)

        # 분할선(빨간색)
        pygame.draw.line(surface, (255, 0, 0), (ax, ay), (bx, by), 1)

        # 확인용: contact point 표시 + normal 화살표(파란색)
        px, py = pygame_coord_from_cartesian(p)
        nx, ny = pygame_coord_from_cartesian(p + n * 50)

        pygame.draw.circle(surface, (255, 255, 0), (int(px), int(py)), 3)
        pygame.draw.line(surface, (0, 0, 255), (px, py), (nx, ny), 2)