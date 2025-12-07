from components.body import Body, Polygon, polygon_area
from components.vector import Vector2D

MAX_FRACTURE_GEN = 4 # 총 4번까지 깨지고 나면 그 조각은 더이상 파쇄 불가.
MIN_AREA = 1.0


def split_polygon_world(
    body: Body, 
    contact_point: Vector2D, 
    normal: Vector2D,
    eps: float = 1e-6,
) -> tuple[list[Vector2D], list[Vector2D]] | None:
    """
    world 좌표 기준 convex polygon을 (p, n) 선으로 둘로 자름.
    성공하면 (polyA_vertices, polyB_vertices)를 반환, 실패하면 None.
    """
    # d: 균열이 진행되는 방향으로 쓰고 싶은 벡터 (법선 방향)
    d = normal.normalize()
    p = contact_point

    # m: 자르는 선의 법선(평면 normal) - d에 수직
    m = Vector2D(-d.y, d.x)


    # 디버그용으로 분할선 정보 저장
    from components.fracture_debug import DEBUG_SPLIT_LINES
    DEBUG_SPLIT_LINES.append((p, d))

    verts = body.get_vertices()   # world 좌표
    count = len(verts)
    if count < 3:
        return None

    front: list[Vector2D] = []
    back: list[Vector2D] = []

    # signed distance는 m으로 계산해야 함
    distances = [(v - p).dot(m) for v in verts]

    for i in range(count):
        j = (i + 1) % count
        v_i = verts[i]
        v_j = verts[j]
        d_i = distances[i]
        d_j = distances[j]

        # 정점 분류 (선 위에 있으면 양쪽에 다 포함시켜도 됨)
        if d_i >= -eps:
            front.append(v_i)
        if d_i <= eps:
            back.append(v_i)

        # 부호가 다른 경우에만 교차점 생성
        if d_i * d_j < -eps * eps:
            t = -d_i / (d_j - d_i)
            t = max(0.0, min(1.0, t))
            intersection = v_i + (v_j - v_i) * t

            front.append(intersection)
            back.append(intersection)

    if len(front) < 3 or len(back) < 3:
        return None

    return front, back

def create_fragment_bodies_from_split(
    body: Body,                  # 원래 바디
    front_verts: list[Vector2D], # 분할선 앞의 정점들
    back_verts: list[Vector2D],  # 분할선 뒤의 정점들
    energy_scale: float = 0.9,   # 쪼갠 후에 에너지 보존률 계수
) -> tuple[Polygon, Polygon]:
    """강체 파쇄 시 vertex 정보를 기반으로 실제 파편을(Polygon 객체) 만들어 냄."""

    # 면적 계산
    area_front = polygon_area(front_verts)
    area_back  = polygon_area(back_verts)
    total_area = area_front + area_back
    if total_area <= 0:
        return None
    
    # 너무 작은 조각이면 포기 (폭주 방지용)
    if area_front < MIN_AREA or area_back < MIN_AREA:
        return None

    # 질량을 면적으로 나누어 가짐.
    mass_front = body.mass * (area_front / total_area)
    mass_back  = body.mass * (area_back  / total_area)

    print(f"Fraction : {area_front / total_area} /  {area_back  / total_area}")

    # centroid 계산 - Polygon 객체 생성자에 넘겨줄 x, y
    def centroid(vertices: list[Vector2D]) -> Vector2D:
        # 단순 area-weighted centroid (convex 가정)
        A = 0.0
        cx = 0.0
        cy = 0.0
        n = len(vertices)
        for i in range(n):
            j = (i + 1) % n
            cross = vertices[i].cross(vertices[j])
            A += cross
            cx += (vertices[i].x + vertices[j].x) * cross
            cy += (vertices[i].y + vertices[j].y) * cross
        A *= 0.5
        if abs(A) < 1e-8:
            # fallback
            return sum(vertices, Vector2D(0, 0)) / n
        cx /= (6.0 * A)
        cy /= (6.0 * A)
        return Vector2D(cx, cy)

    # 글로벌 좌표계 상의 위치를 여기서 propagate 해야 함. (centroid를 객체 생성 후에 계산하면 너무 늦는다.)
    c_front = centroid(front_verts)
    c_back  = centroid(back_verts)

    # Polygon 객체 생성
    frag1 = Polygon(
        c_front.x, c_front.y,
        front_verts,
        mass=mass_front,
        bounce=body.bounce,
        name=(body.name or "") + "_f1",
        is_static=body.is_static,
        ignore_gravity=body.ignore_gravity,
        color=body.color,
        linear_damp=body.linear_damp,
        angular_damp=body.angular_damp,
        is_breakable=body.is_breakable if body.fracture_gen < MAX_FRACTURE_GEN else False,
        break_impulse=body.break_impulse,
        fracture_gen = body.fracture_gen + 1
    )
    frag2 = Polygon(
        c_back.x, c_back.y,
        back_verts,
        mass=mass_back,
        bounce=body.bounce,
        name=(body.name or "") + "_f2",
        is_static=body.is_static,
        ignore_gravity=body.ignore_gravity,
        color=body.color,
        linear_damp=body.linear_damp,
        angular_damp=body.angular_damp,
        is_breakable=body.is_breakable if body.fracture_gen < MAX_FRACTURE_GEN else False,
        break_impulse=body.break_impulse,
        fracture_gen = body.fracture_gen + 1
    )

    # 속도/각속도는 부모 걸 약간만 줄여서 복사
    frag1.velocity = body.velocity * energy_scale
    frag2.velocity = body.velocity * energy_scale
    frag1.angular_velocity = body.angular_velocity * energy_scale
    frag2.angular_velocity = body.angular_velocity * energy_scale

    return frag1, frag2



pending_fractures: list[tuple[Body, Vector2D, Vector2D]] = []

def request_fracture(body: Body, p: Vector2D, n: Vector2D):
    """다음 Frame에서 실제 파쇄가 일어날 수 있도록 보류."""
    pending_fractures.append((body, p, n))