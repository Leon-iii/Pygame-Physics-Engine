from components.body import Body
from components.collision import collide
from components.constraint import DistanceConstraint
from components.vector import Vector2D
from components.bvh import BVHNode, build_bvh, collect_pairs, draw_bvh_node
from components.fracture import pending_fractures, split_polygon_world, create_fragment_bodies_from_split

import pygame


# 여러 개의 물리 Body를 관리하면서,
# - 중력 적용
# - 위치/회전 업데이트
# - Body 간 충돌 처리
# 를 한 번에 담당하는 물리 월드(Scene) 클래스
class Scene:
    def __init__(self, bodies: list[Body], gravity=9.8,
                 linear_damping: float = 1.0,
                 angular_damping: float = 1.0):
        # 시뮬레이션 대상이 되는 물체 리스트
        self.bodies: list[Body] = bodies

        # 이번 프레임에 계산된 접촉점들 (디버깅/렌더링용)
        self._contact_points = []

        # 중력 가속도(기본값 9.8)
        self.gravity = gravity

        # 1초당 어느 정도 비율로 속도를 깎을지 결정하는 계수들 (단위: 1/s)
        self.linear_damping = linear_damping
        self.angular_damping = angular_damping

        self.constraints: list[DistanceConstraint] = []

        # BVH 루트 노드.
        self.bvh_root: BVHNode | None = None

    # 새로운 물체를 씬에 추가
    def add(self, *bodies: Body):
        """Body를 한 개 또는 여러 개 받아서 전부 self.bodies에 추가"""
        for body in bodies:
            self.bodies.append(body)

    #####  
    # 각 Body에 중력을 적용해 속도를 변화시키는 함수
    #   v_y += g * dt (이 구현에서는 y축이 위/아래 중 어떤 방향인지에 따라 부호가 달라짐)
    def simulate_gravity(self, dt):
        for body in self.bodies:
            if body.is_static == False and not body.ignore_gravity:
                # 정적 물체가 아니면 중력으로 인한 y방향 속도 변화 적용
                body.velocity[1] -= self.gravity * body.mass * dt

    # 속도(선형/각속도)에 따라 물체의 위치와 각도를 갱신하는 함수
    def update_position(self, dt):
        for body in self.bodies:
            if body.is_static == False:  # 정적 물체는 움직이지 않음
                # 선형 속도에 따른 중심 위치 이동
                body.center += body.velocity * dt
                # 각속도에 따른 회전 각도 변화
                body.angle += body.angular_velocity * dt

    def update_position_rk4(self, dt: float):
        """RK4로 각 Body의 위치/속도, 각도/각속도를 적분해서 업데이트"""
        for body in self.bodies:
            if body.is_static:
                continue

            # 현재 상태
            x0 = body.center              # 위치
            v0 = body.velocity            # 속도
            theta0 = body.angle           # 각도
            omega0 = body.angular_velocity  # 각속도

            # 주어진 상태에서의 도함수(미분값)를 계산하는 내부 함수
            # dx/dt, dv/dt, dθ/dt, dω/dt 를 반환
            def deriv(pos: Vector2D,
                      vel: Vector2D,
                      theta: float,
                      omega: float):

                # 연속 힘/가속도 모델:
                #   - 여기서는 중력만 가속도로 취급 (충돌/제약은 임펄스로 따로 처리)
                #   - 중력이 아래 방향이라고 가정: (0, -g)
                #     필요하면 self.gravity, 축 방향 등 바꿔도 됨
                a = Vector2D(0, -self.gravity)  # dv/dt
                alpha = 0.0                     # dω/dt (회전 가속도는 일단 없음)

                dxdt = vel
                dvdt = a
                dthetadt = omega
                domegadt = alpha

                return dxdt, dvdt, dthetadt, domegadt

            # k1
            k1_x, k1_v, k1_th, k1_w = deriv(x0, v0, theta0, omega0)

            # k2: t + dt/2, x + k1*dt/2
            k2_x, k2_v, k2_th, k2_w = deriv(
                x0 + k1_x * (dt * 0.5),
                v0 + k1_v * (dt * 0.5),
                theta0 + k1_th * (dt * 0.5),
                omega0 + k1_w * (dt * 0.5),
            )

            # k3: t + dt/2, x + k2*dt/2
            k3_x, k3_v, k3_th, k3_w = deriv(
                x0 + k2_x * (dt * 0.5),
                v0 + k2_v * (dt * 0.5),
                theta0 + k2_th * (dt * 0.5),
                omega0 + k2_w * (dt * 0.5),
            )

            # k4: t + dt, x + k3*dt
            k4_x, k4_v, k4_th, k4_w = deriv(
                x0 + k3_x * dt,
                v0 + k3_v * dt,
                theta0 + k3_th * dt,
                omega0 + k3_w * dt,
            )

            # 가중 평균 (1, 2, 2, 1)
            factor = dt / 6.0

            dx = (k1_x + k2_x * 2 + k3_x * 2 + k4_x) * factor
            dv = (k1_v + k2_v * 2 + k3_v * 2 + k4_v) * factor
            dtheta = (k1_th + 2 * k2_th + 2 * k3_th + k4_th) * factor
            domega = (k1_w + 2 * k2_w + 2 * k3_w + k4_w) * factor

            # 최종 상태 업데이트
            body.center = x0 + dx
            body.velocity = v0 + dv
            body.angle = theta0 + dtheta
            body.angular_velocity = omega0 + domega

    # BVH로부터 받은 충돌 후보로부터 실제 충돌 여부를 확인하고, 충돌 반응까지 처리
    def handle_collisions(self):
        self._contact_points = []

        # 1) 매 프레임 BVH 빌드
        self.bvh_root = build_bvh(self.bodies)

        if self.bvh_root is None:
            return

        # 2) BVH에서 broad-phase 후보 쌍 수집
        candidate_pairs = collect_pairs(self.bvh_root)

        # 3) 후보 쌍들에 대해서만 실제 충돌/반응 실행
        for body_1, body_2 in candidate_pairs:
            contact_points = collide(body_1, body_2)
            if contact_points is None:
                continue

            for point in contact_points:
                if point is None:
                    continue
                self._contact_points.append(point)

    
    def apply_damping(self, dt: float):
        for body in self.bodies:
            if body.is_static:
                continue
            
            # 강체가 감쇠상수를 가지고 있으면 그걸 사용,
            # 없으면 전역 감쇠상수 (Scene) 사용
            if body.linear_damp == None: ld = self.linear_damping
            else:                        ld = body.linear_damp

            if body.angular_damp == None: ad = self.angular_damping
            else:                         ad = body.angular_damp

            # 0 ~ 1 사이로 클램프
            linear_factor = max(0.0, 1.0 - ld * dt)
            angular_factor = max(0.0, 1.0 - ad * dt)

            body.velocity *= linear_factor
            body.angular_velocity *= angular_factor


    def add_constraint(self, constraint: DistanceConstraint):
        self.constraints.append(constraint)

    def solve_constraints(self, iterations: int = 5):
        # 여러 번 반복해서 제약을 조금씩 수렴시키는 방식 (PBD 스타일)

        for _ in range(iterations):
            for c in self.constraints:
                if not c.broken:
                    c.solve()
                else:
                    self.constraints.remove(c)


    # 한 타임스텝(dt)만큼 시뮬레이션을 진행하는 함수
    def step(self, dt):
        
        # 1) 중력 적용
        self.simulate_gravity(dt)

        # 2) 속도에 먼저 댐핑 적용
        self.apply_damping(dt)

        # 3) 속도/각속도로 위치/각도 업데이트
        self.update_position_rk4(dt)

        # 4) 충돌 처리 (임펄스)
        self.handle_collisions()

        # 5) 제약(로프 등) 해결
        self.solve_constraints()
        
        # 6) 파쇄 예정인 강체 파쇄 -> 다음 프레임에 반영.
        self.apply_pending_fractures()


    def apply_pending_fractures(self):

        for body, p, n in list(pending_fractures):
            if body not in self.bodies:
                continue
            if not isinstance(body, Body):
                continue

            result = split_polygon_world(body, p, n)
            if result is None:
                continue

            front_verts, back_verts = result
            frag1, frag2 = create_fragment_bodies_from_split(body, front_verts, back_verts)

            # 원본 삭제 + 조각 추가
            self.bodies.remove(body)
            self.bodies.extend([frag1, frag2])

        pending_fractures.clear()

    def debug_draw_bvh(self, surface: "pygame.Surface", max_depth: int | None = None):
        """현재 BVH를 화면에 디버그용으로 그린다."""
        if self.bvh_root is None:
            return

        draw_bvh_node(surface, self.bvh_root, depth=0, max_depth=max_depth)
        # 루트만 그리면 재귀 호출하면서 알아서 자식까지 그려짐.
