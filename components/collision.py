from components.vector import Vector2D
from components.body import Body, Polygon, Rectangle, Circle
from components.fracture import request_fracture


# 두 물체의 충돌을 판정하고, 침투 깊이/법선 및 접촉점에 따라 반응(속도/각속도 수정)까지 처리하는 함수
def collide(body_1: Body, body_2: Body, include_rotation=True):
    # 1단계: 도형 타입에 따라 적절한 충돌 판정 함수를 호출해서 법선 벡터와 침투 깊이 계산
    if isinstance(body_1, (Rectangle, Polygon)) and isinstance(body_2, (Rectangle, Polygon)):
        normal, depth = polygons_collision(body_1, body_2)
    elif isinstance(body_1, Circle) and isinstance(body_2, Circle):
        normal, depth = circles_collision(body_1, body_2)
    elif isinstance(body_1, (Rectangle, Polygon)) and isinstance(body_2, Circle):
        normal, depth = polygon_circle_collision(body_1, body_2)
    elif isinstance(body_1, Circle) and isinstance(body_2, (Rectangle, Polygon)):
        normal, depth = polygon_circle_collision(body_2, body_1)

    # 충돌이 없으면(법선/깊이가 None) 종료
    if normal is None or depth is None:
        return

    # 2단계: 동일한 타입 분류에 맞춰 실제 접촉점(contact point) 계산
    if isinstance(body_1, (Rectangle, Polygon)) and isinstance(body_2, (Rectangle, Polygon)):
        contact_points = polygons_contact_points(body_1, body_2)
    elif isinstance(body_1, Circle) and isinstance(body_2, Circle):
        contact_points = circles_contact_points(body_1, body_2)
    elif isinstance(body_1, (Rectangle, Polygon)) and isinstance(body_2, Circle):
        contact_points = polygon_circle_contact_points(body_1, body_2)
    elif isinstance(body_1, Circle) and isinstance(body_2, (Rectangle, Polygon)):
        contact_points = polygon_circle_contact_points(body_2, body_1)
        # 매개변수 순서를 바꿔 호출했으므로 법선 방향을 다시 반전
        normal = -normal

    # 3단계: 회전 효과 포함 여부에 따라 다른 반응 함수 호출
    if include_rotation:
        response_with_rotation(body_1, body_2, normal, depth, contact_points)
    else:
        response(body_1, body_2, normal, depth)

    # 추후 디버그/렌더링용으로 접촉점 반환
    return contact_points


# 회전(각속도)을 무시하고 선형 속도만으로 충돌 반응을 처리하는 함수
def response(body_1: Body, body_2: Body,
             normal_vector: Vector2D,
             penetration_depth: float):
    normal_vector *= -1
    separate_bodies(body_1, body_2, normal_vector, penetration_depth)

    relative_velocity = body_2.velocity - body_1.velocity
    penetration_velocity = relative_velocity.dot(normal_vector)

    if penetration_velocity > 0:
        return

    r = max(body_1.bounce, body_2.bounce)
    j = -(1 + r) * penetration_velocity
    j /= 1 / body_1.mass + 1 / body_2.mass

    impulse = normal_vector * j

    # 회전 안 보려면 r=0으로 넣어도 되고, 그냥 직접 velocity만 수정해도 됨
    body_1.apply_impulse(-impulse, Vector2D(0, 0))
    body_2.apply_impulse( impulse, Vector2D(0, 0))


# 회전(각속도 포함)을 고려한 충돌 반응 함수
# 접촉점에서의 상대 속도를 기준으로 선형 + 각속도에 모두 임펄스를 적용
def response_with_rotation(body_1: Body, body_2: Body,
                           normal_vector: Vector2D,
                           penetration_depth: float,
                           contact_point: list[Vector2D]):

    # 둘 다 정적이면 아무것도 하지 않음
    if body_1.is_static and body_2.is_static:
        return
    
    normal_vector *= -1

    # 1단계: 물체 분리 (겹침 해소)
    separate_bodies(body_1, body_2, normal_vector, penetration_depth)

    # 2단계: 접촉점이 여러 개라면 평균 위치를 사용, 하나면 그대로 사용
    if len(contact_point) == 2:
        contact_point = (contact_point[0] + contact_point[1]) / 2
    else:
        contact_point = contact_point[0]

    # 3단계: 각 물체 중심에서 접촉점까지의 벡터 r1, r2
    r_1 = contact_point - body_1.center
    r_2 = contact_point - body_2.center

    # 4단계: 2D에서 각속도에 의한 속도를 계산하기 위한 수직 벡터
    r_1_perp = Vector2D(-r_1.y, r_1.x)
    r_2_perp = Vector2D(-r_2.y, r_2.x)

    # 5단계: 접촉점에서의 상대 속도 (선속도 + 각속도 성분을 모두 포함)
    relative_velocity = (body_2.velocity + r_2_perp * body_2.angular_velocity) - (
        body_1.velocity + r_1_perp * body_1.angular_velocity
    )
    penetration_velocity = relative_velocity.dot(normal_vector)

    # 서로 멀어지는 중이면 임펄스 계산 불필요
    if penetration_velocity > 0:
        return

    # 6단계: 회전 관성까지 고려한 임펄스 스칼라 j 계산
    r = max(body_1.bounce, body_2.bounce)
    j = -(1 + r) * penetration_velocity
    j /= (
        1 / body_1.mass
        + 1 / body_2.mass
        + (r_1_perp.dot(normal_vector) ** 2) / body_1.inertia
        + (r_2_perp.dot(normal_vector) ** 2) / body_2.inertia
    )

    # 7단계: 임펄스 벡터
    impulse = normal_vector * j

    # 여기부턴 "얼마나 때릴지"가 정해졌고, 실제 적용은 Body 쪽 메서드로 위임
    body_1.apply_impulse(-impulse, r_1)
    body_2.apply_impulse( impulse, r_2)

    # 파쇄.
    if body_1.is_breakable and abs(j) > body_1.break_impulse:
        request_fracture(body_1, contact_point, normal_vector)

    if body_2.is_breakable and abs(j) > body_2.break_impulse:
        # body_2 기준으로는 법선 방향 반대
        request_fracture(body_2, contact_point, -normal_vector)


############################################################################################################################################################


# 원(센터, 반지름)을 주어진 축(axis)에 정사영하여 최소/최대 스칼라 값을 반환
def project_circle(center, radius: float, axis: Vector2D):
    # 축 방향 단위벡터
    direction = axis.normalize()
    direction_and_radius = direction * radius

    # 축 방향/반대 방향의 점 두 개
    p1 = center + direction_and_radius
    p2 = center - direction_and_radius

    # 두 점을 축(axis)에 투영한 스칼라 값
    min_proj = p1.dot(axis)
    max_proj = p2.dot(axis)

    # 순서 보정
    if min_proj > max_proj:
        min_proj, max_proj = max_proj, min_proj

    return min_proj, max_proj

def bodies_collision(body_1: Body, body_2: Body):
    normal = Vector2D(0, 0)       # 최소 침투축 법선
    depth = float('inf')          # 최소 침투 깊이(초기값 무한대)

# SAT(Separating Axis Theorem)을 이용한 다각형-다각형 충돌 판정
# 충돌 시: 최소 침투축의 법선 벡터와 침투 깊이 반환, 아니면 (None, None)
def polygons_collision(polygon_1: Polygon, polygon_2: Polygon):
    normal = Vector2D(0, 0)       # 최소 침투축 법선
    depth = float('inf')          # 최소 침투 깊이(초기값 무한대)

    vertices1 = polygon_1.get_vertices()
    vertices2 = polygon_2.get_vertices()

    # 1단계: polygon_1의 모든 변에서 법선 축을 구해 투영
    for i in range(len(vertices1)):
        va = vertices1[i]
        vb = vertices1[(i + 1) % len(vertices1)]
        edge = vb - va
        # 변에 수직인 단위벡터를 분리축으로 사용
        axis = Vector2D(-edge.y, edge.x).normalize()

        # 두 다각형을 같은 축에 투영
        min_a, max_a = project_vertices(polygon_1.get_vertices(), axis)
        min_b, max_b = project_vertices(polygon_2.get_vertices(), axis)

        # 투영 구간이 겹치지 않으면 분리축 존재 → 충돌 없음
        if min_a >= max_b or min_b >= max_a:
            return None, None

        # 겹치는 길이(침투 깊이 후보)
        axis_depth = min(max_b - min_a, max_a - min_b)

        # 더 작은 침투 깊이를 가진 축을 기록
        if axis_depth < depth:
            depth = axis_depth
            normal = axis

    # 2단계: polygon_2의 변에서도 동일 작업 (모든 후보 축 검사)
    for i in range(len(vertices2)):
        va = vertices2[i]
        vb = vertices2[(i + 1) % len(vertices2)]
        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()

        min_a, max_a = project_vertices(polygon_1.get_vertices(), axis)
        min_b, max_b = project_vertices(polygon_2.get_vertices(), axis)

        if min_a >= max_b or min_b >= max_a:
            return None, None

        axis_depth = min(max_b - min_a, max_a - min_b)

        if axis_depth < depth:
            depth = axis_depth
            normal = axis

    # 3단계: 법선 방향 정규화 (polygon_2 → polygon_1 방향이 되도록)
    direction = (polygon_1.center - polygon_2.center).normalize()
    if direction.dot(normal) < 0:
        normal *= -1

    return normal, depth

# 원-원 충돌 판정 및 법선/침투 깊이 계산 함수
def circles_collision(body_1: Circle, body_2: Circle):

    # 두 중심 사이 거리
    distance = Vector2D.distance(body_1.center, body_2.center)

    # 중심 거리 >= 반지름 합 → 서로 닿지 않음
    if distance >= body_1.radius + body_2.radius:
        return None, None

    # 법선: body_2 → body_1 방향 (이 구현에서는 이렇게 정의)
    normal_vector = (body_1.center - body_2.center).normalize()

    # 침투 깊이 = 반지름 합 - 중심 거리
    penetration_depth = body_1.radius + body_2.radius - distance

    return normal_vector, penetration_depth

# SAT를 이용한 다각형-원 충돌 판정
# 1) 다각형 변의 법선 축, 2) 원 중심에서 가장 가까운 꼭짓점 방향 축 두 종류를 검사
def polygon_circle_collision(polygon: Polygon, circle: Circle):

    normal = Vector2D(0, 0)
    penetration_depth = float('inf')

    vertices = polygon.get_vertices()

    # 1단계: 다각형의 각 변에서 분리축(법선)을 구해 원과 함께 투영
    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]

        edge = vb - va
        axis = Vector2D(-edge.y, edge.x).normalize()

        # 다각형과 원을 동일한 축에 투영
        min_a, max_a = project_vertices(vertices, axis)
        min_b, max_b = project_circle(circle.center, circle.radius, axis)

        # 투영 구간이 분리되면 충돌 없음
        if max_a <= min_b or max_b <= min_a:
            return None, None

        axis_depth = min(max_b - min_a, max_a - min_b)

        if axis_depth < penetration_depth:
            penetration_depth = axis_depth
            normal = axis

    # 2단계: 원 중심에서 가장 가까운 다각형 꼭짓점을 향하는 축도 검사
    cp_index = find_closest_point_on_polygon(circle.center, vertices)
    cp = vertices[cp_index]
    axis = (cp - circle.center).normalize()

    min_a, max_a = project_circle(circle.center, circle.radius, axis)
    min_b, max_b = project_vertices(vertices, axis)

    if max_a <= min_b or max_b <= min_a:
        return None, None

    axis_depth = min(max_b - min_a, max_a - min_b)

    if axis_depth < penetration_depth:
        penetration_depth = axis_depth
        normal = axis

    # 3단계: 법선 방향 정규화 (circle → polygon 방향이 되도록)
    direction = (polygon.center - circle.center).normalize()
    if direction.dot(normal) < 0:
        normal *= -1

    return normal, penetration_depth


# 정점 리스트를 주어진 축(axis)에 정사영하여 최소/최대 스칼라 값을 구하는 함수
def project_vertices(vertices: list[Vector2D], axis: Vector2D):
    min_proj = float('inf')
    max_proj = float('-inf')

    for v in vertices:
        proj = v.dot(axis)

        if proj < min_proj:
            min_proj = proj
        if proj > max_proj:
            max_proj = proj

    return min_proj, max_proj


# 원의 중심에서 가장 가까운 다각형 꼭짓점의 인덱스를 찾는 함수
def find_closest_point_on_polygon(circle_center: Vector2D, vertices: list[Vector2D]):
    result = -1
    min_distance = float('inf')

    for i, v in enumerate(vertices):
        dist = Vector2D.distance(v, circle_center)

        if dist < min_distance:
            min_distance = dist
            result = i

    return result


# 침투 깊이에 비례해서 두 물체의 중심을 이동시켜 겹침을 해소하는 함수
def separate_bodies(body_1: Body, body_2: Body, normal, penetration_depth):
    # 단순히 법선 방향 * 침투 깊이만큼 두 물체를 벌려 줌
    separation_vector = normal * penetration_depth

    # 다른 개선된 분리 방법(질량 비례 이동 등)이 주석으로 남아 있음
    # percent = 0.2; slope = 0.01
    # separation_vector = max(penetration_depth - slope, 0) / (1 / body_1.mass + 1 / body_2.mass) * percent * normal

    if body_1.is_static:
        # body_1이 고정되어 있으면 body_2만 이동
        body_2.center += separation_vector
    elif body_2.is_static:
        # body_2가 고정이면 body_1만 이동
        body_1.center -= separation_vector
    else:
        # 둘 다 움직일 수 있으면 절반씩 나눠서 이동
        body_1.center -= separation_vector / 2
        body_2.center += separation_vector / 2

    # 질량 비례 분리 방법 예시
    # if body_1.is_static == False:
    #     body_1.center -= separation_vector * 1 / body_1.mass
    # if body_2.is_static == False:
    #     body_2.center += separation_vector * 1 / body_2.mass


# 한 점을 선분 AB 위로 직투영시키고, 그 투영점과 원래 점 사이의 거리까지 반환하는 함수
def point_to_line_segment_projection(point: Vector2D, a: Vector2D, b: Vector2D):
    ab = b - a
    ap = point - a

    # 스칼라 투영값
    proj = ap.dot(ab)
    d = proj / ab.dot(ab)  # 0~1 사이면 선분 내부, 아니면 바깥쪽

    # d를 0~1 구간으로 clamp하면서 실제 접촉점 결정
    if d <= 0:
        contact_point = a
    elif d >= 1:
        contact_point = b
    else:
        contact_point = a + ab * d

    # 원래 점과 투영점 사이의 거리
    distance = Vector2D.distance(contact_point, point)

    return contact_point, distance


# 다각형-다각형 충돌에서 실제 접촉점(1~2개)을 찾는 함수
# 각 꼭짓점 ↔ 다른 다각형의 변에 대한 최단 거리 투영을 이용
def polygons_contact_points(polygon_1: Polygon, polygon_2: Polygon):
    epsilon = 0.0005         # 거의 같은 거리로 볼 기준
    min_distance = float('inf')
    contact_point_1 = None
    contact_point_2 = None

    # 1단계: polygon_1의 각 정점을 polygon_2의 모든 변에 투영해서 최단 거리 접촉점을 찾음
    for i in range(len(polygon_1.get_vertices())):
        vp = polygon_1.get_vertices()[i]
        for j in range(len(polygon_2.get_vertices())):
            va = polygon_2.get_vertices()[j]
            vb = polygon_2.get_vertices()[(j + 1) % len(polygon_2.get_vertices())]

            cp, distance = point_to_line_segment_projection(vp, va, vb)

            # 기존 최단 거리와 거의 같고, 기존 접촉점과도 충분히 떨어져 있으면 두 번째 접촉점으로 등록
            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            # 더 짧은 거리라면 새 접촉점으로 갱신, 두 번째 접촉점 초기화
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    # 2단계: 방향을 반대로 해서 polygon_2의 정점도 polygon_1의 변에 투영
    for i in range(len(polygon_2.get_vertices())):
        vp = polygon_2.get_vertices()[i]
        for j in range(len(polygon_1.get_vertices())):
            va = polygon_1.get_vertices()[j]
            vb = polygon_1.get_vertices()[(j + 1) % len(polygon_1.get_vertices())]

            cp, distance = point_to_line_segment_projection(vp, va, vb)

            if contact_point_1 is not None and abs(distance - min_distance) < epsilon and not cp.distance_to(contact_point_1) < epsilon:
                contact_point_2 = cp
            elif distance < min_distance:
                min_distance = distance
                contact_point_2 = None
                contact_point_1 = cp

    # None이 아닌 접촉점만 리스트로 반환
    return [cp for cp in [contact_point_1, contact_point_2] if cp is not None]


# 다각형-원 충돌에서 접촉점(1개)을 찾는 함수
# 원 중심을 각 변에 투영해 가장 가까운 점을 접촉점으로 사용
def polygon_circle_contact_points(polygon: Polygon, circle: Circle):
    min_distance = float('inf')
    vertices = polygon.get_vertices()

    for i in range(len(vertices)):
        va = vertices[i]
        vb = vertices[(i + 1) % len(vertices)]

        cp, distance = point_to_line_segment_projection(circle.center, va, vb)

        if distance < min_distance:
            min_distance = distance
            contact_point = cp

    return [contact_point]


# 원-원 충돌의 접촉점 계산 함수
def circles_contact_points(body_1: Circle, body_2: Circle):
    # body_1 → body_2 방향의 단위 법선
    normal = (body_2.center - body_1.center).normalize()
    # body_1의 중심에서 반지름만큼 normal 방향으로 이동한 점이 접촉점
    contact_point = body_1.center + normal * body_1.radius

    return [contact_point]