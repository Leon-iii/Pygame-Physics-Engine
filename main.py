import pygame
import sys
from components.body import Body, Circle, Rectangle, Polygon
from components.scene import Scene
from components.constraint import DistanceConstraint
from components.vector import Vector2D
from components.utils import cartesian_coord_from_pygame, pygame_coord_from_cartesian

WIDTH, HEIGHT = 1000, 800
PLAYER_SPEED_X = PLAYER_SPEED_Y = 3
FPS = 360
GRAVITY = 35

SHOW_COLLISION_POINTS = False
SHOW_FRACTURE_NORMALS = False
SHOW_BVH_TREE = False

COLORS = {
    "white": (255, 255, 255),
    "black": (0, 0, 0),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "orange": (255, 128, 0),
    "cyan": (0, 255, 255),
    "navy": (0, 0, 128),
    
    "yellow": (255, 255, 0),
    "purple": (128, 0, 128),
    "pink": (255, 192, 203),
    "brown": (165, 42, 42),
    "gray": (128, 128, 128),
    "gold": (255, 215, 0),
    "silver": (192, 192, 192),
    "maroon": (128, 0, 0),
    "olive": (128, 128, 0),
    "teal": (102, 178, 178),
    "lime": (0, 255, 0),
    "indigo": (75, 0, 130)
}


# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Physics Engine")


Scene = Scene(
    [ ],
    GRAVITY,
)

Scene.add(
    Rectangle(
        x=200,
        y=100,
        width=60,
        height=50,
        mass=100,
        is_breakable=True,
        break_impulse=100000
        ),
    Rectangle(
        x=200,
        y=500,
        width=60,
        height=50,
        mass=100,
        is_breakable=False,
        ignore_gravity=True
        ),
    Circle(
        x=WIDTH / 2 + 200,
        y=300,
        radius=50,
        mass=300,
        ),

    Rectangle(
        x=WIDTH / 2,
        y=10,
        width=WIDTH,
        height=20,
        is_static=True,
        bounce=0.7,
        name="Ground"
        ),

    Rectangle(
        x=WIDTH / 2,
        y=HEIGHT-10,
        width=WIDTH,
        height=20,
        is_static=True,
        bounce=0.7,
        name="Ceiling"
        ),
    Rectangle(
        x=10,
        y=HEIGHT / 2,
        width=20,
        height=HEIGHT,
        is_static=True,
        bounce=0.7,
        name="LeftWall"
        ),
    Rectangle(
        x=WIDTH - 10,
        y=HEIGHT / 2,
        width=20,
        height=HEIGHT,
        is_static=True,
        bounce=0.7,
        name="RightWall"
        ),
)

polygon_1 = Polygon(
        x=400,
        y=250,
        vertices=[(0, 0), (50, 0), (50, 25), (25, 100), (0, 50)],
        mass=100
    )
polygon_1.rotate(90, in_radians=False)
Scene.add(polygon_1)


cursor = Circle(
    x=0,
    y=0,
    radius=15,
    mass=100,
    color=COLORS["indigo"],
    ignore_gravity = True,
    linear_damp=7,
    angular_damp=5
)

Scene.add(cursor)

cursor.teleport(cartesian_coord_from_pygame(pygame.mouse.get_pos()))


######################### 로프 생성 로직 ##########################

rope_1 = None
rope_2 = None

rope_length = 15

for idx in range(rope_length):
    rope_1 = rope_2
    rope_2 = Circle(
        x=400 + idx * 10,
        y=700,
        radius=5,
        mass=10,
        color=COLORS["black"] if idx == 0 else COLORS["brown"],
        is_static= True if idx == 0 else False # 첫 번쨰 로프는 스태틱.
    ) if idx < rope_length - 1 else None

    if rope_2 != None: Scene.add(rope_2)

    if rope_1 != None and rope_2 != None:
        Scene.add_constraint(
            DistanceConstraint(
                rope_1, rope_2,
                Vector2D(), Vector2D(),
                rest_length=15,
                stiffness=0.5,
                break_distance=30,
                draw=True, draw_color=COLORS["brown"], draw_width=10
            )
        )

##################################################################

########################## 젤리 큐브 생성 로직 ##########################

import math

jelly_vertices = []

rows = 5
cols = 5
spacing = 20               # 점 간격 (x, y 둘 다 15씩)
rest_len = spacing         # 가로/세로 스프링 자연 길이
rest_len_diag = spacing * math.sqrt(2)  # 대각선 자연 길이, 약 21.21

k_inner = 0.4   # 속살 stiffness
k_outer = 0.8   # 껍질 stiffness

def spring_k_for(row, col, rows, cols):
    if row == 0 or row == rows - 1 or col == 0 or col == cols - 1:
        return k_outer
    return k_inner

for row in range(rows):
    cur_row = []
    for col in range(cols):
        new_vertex = Circle(
            x=600 + row * spacing,
            y=600 + col * spacing,
            radius=5,
            mass=5,
            bounce=1.0,
            color=COLORS["olive"]
        )
        cur_row.append(new_vertex)
        Scene.add(new_vertex)

        k = spring_k_for(row, col, rows, cols)

        # 가로 스프링
        if col > 0:
            Scene.add_constraint(
                DistanceConstraint(
                    cur_row[col - 1], cur_row[col],
                    Vector2D(), Vector2D(),
                    rest_length=rest_len,
                    stiffness=k,
                    break_distance=rest_len * 2,  # 적당히 여유
                    draw=True if row == 0 or row == rows - 1 else False,
                    draw_color=COLORS["lime"], draw_width=10
                )
            )

        # 세로 스프링
        if row > 0:
            Scene.add_constraint(
                DistanceConstraint(
                    jelly_vertices[row - 1][col], cur_row[col],
                    Vector2D(), Vector2D(),
                    rest_length=rest_len,
                    stiffness=k,
                    break_distance=rest_len * 2,
                    draw=True if col == 0 or col == cols - 1 else False,
                    draw_color=COLORS["lime"], draw_width=10
                )
            )

        # \ 대각선
        if row > 0 and col > 0:
            Scene.add_constraint(
                DistanceConstraint(
                    jelly_vertices[row - 1][col - 1], cur_row[col],
                    Vector2D(), Vector2D(),
                    rest_length=rest_len_diag,
                    stiffness=k,
                    break_distance=rest_len_diag * 2,
                    draw=True,
                    draw_color=COLORS["olive"], draw_width=3
                )
            )

        # / 대각선
        if row > 0 and col < cols - 1:
            Scene.add_constraint(
                DistanceConstraint(
                    jelly_vertices[row - 1][col + 1], cur_row[col],
                    Vector2D(), Vector2D(),
                    rest_length=rest_len_diag,
                    stiffness=k,
                    break_distance=rest_len_diag * 2,
                    draw=True,
                    draw_color=COLORS["olive"], draw_width=3
                )
            )

    jelly_vertices.append(cur_row)


#######################################################################


move_left = False
move_right = False
move_up = False
move_down = False
dt = 1 / FPS  

# Main game loop
while True:
    # 이벤트 처리
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # 커서 방향으로 강체를 이동시킴
    mouse_x, mouse_y = pygame.mouse.get_pos()
    vec = Vector2D(mouse_x, HEIGHT - mouse_y) - cursor.center
    cursor.apply_impulse(cursor.mass * vec, Vector2D(0, 0))

    Scene.step(dt)
    screen.fill(COLORS["white"])

    if SHOW_BVH_TREE:
        Scene.debug_draw_bvh(screen)

    if SHOW_FRACTURE_NORMALS:
        from components.fracture_debug import debug_draw_split_lines
        debug_draw_split_lines(screen)


    for body in Scene.bodies:
        
        color = COLORS["navy"]

        if body.is_static:    color = COLORS["black"]
        if body.is_breakable: color = COLORS["teal"]

        if body.color != None: color = body.color

        if isinstance(body, (Rectangle, Polygon)):
            
            pygame.draw.polygon(
                screen,
                color,
                [(vertex.x, HEIGHT - vertex.y) for vertex in body.get_vertices()],
            )
        elif isinstance(body, Circle):
            
            pygame.draw.circle(
                screen, color, (body.center[0], HEIGHT - body.center[1]), body.radius
            )

    for constr in Scene.constraints:
        if constr.draw:
            pygame.draw.line(screen, constr.draw_color,
                             pygame_coord_from_cartesian(constr.body_a.center),
                             pygame_coord_from_cartesian(constr.body_b.center),
                             constr.draw_width)

    if SHOW_COLLISION_POINTS:        
        for point in Scene._contact_points:
            pygame.draw.circle(screen, COLORS["green"], (point.x, HEIGHT - point.y), 4)

    pygame.display.flip()

    pygame.time.Clock().tick(FPS)