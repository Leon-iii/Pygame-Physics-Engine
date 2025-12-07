from components.vector import Vector2D
from components.body import Body

class DistanceConstraint:
    def __init__(
        self,
        body_a: Body,
        body_b: Body,
        anchor_a: Vector2D,   # body_a 로컬 좌표계 기준 앵커
        anchor_b: Vector2D,   # body_b 로컬 좌표계 기준 앵커
        rest_length: float | None = None, # Union Type Hint : 값 주입 시 float, 값 주입하지 않을 시 None
        stiffness: float = 1.0,      # 0~1, 1이면 완전히 강체에 가깝게
        break_distance: float = float("inf"),  # 얼마나 늘어나면 끊어질지 - softbody 파괴?
        draw: bool = False,
        draw_color: tuple[int, int, int] | None = None, # 화면에 그릴 색상
        draw_width: float = 1.0
    ):
        self.body_a = body_a
        self.body_b = body_b
        self.anchor_a_local = anchor_a
        self.anchor_b_local = anchor_b
        self.stiffness = stiffness
        self.break_distance = break_distance

        self.draw = draw
        self.draw_color = draw_color
        self.draw_width = draw_width

        # 초기 길이를 기본 rest_length로 사용
        if rest_length is None:
            pa = self._anchor_world_a()
            pb = self._anchor_world_b()
            self.rest_length = Vector2D.distance(pa, pb)
        else:
            self.rest_length = rest_length

        self.broken = False

    def _anchor_world_a(self) -> Vector2D:
        # 로컬 앵커를 바디의 각도/위치를 반영해서 월드 좌표로 변환
        return self.anchor_a_local.rotate(self.body_a.angle).add(self.body_a.center)

    def _anchor_world_b(self) -> Vector2D:
        return self.anchor_b_local.rotate(self.body_b.angle).add(self.body_b.center)

    def solve(self):
        if self.broken:
            return

        # 월드 좌표계에서 두 앵커 위치
        pa = self._anchor_world_a()
        pb = self._anchor_world_b()

        delta = pb - pa
        dist = delta.magnitude()

        if dist == 0:
            return

        # 지금 길이가 rest_length에서 얼마나 벗어났는지 (위치 오차)
        diff = dist - self.rest_length

        # 얼마나 찢어졌는지 기준을 넘으면 제약을 끊어버림
        if abs(diff) > self.break_distance:
            self.broken = True
            print("constraint broken")
            return

        # 제약 방향 (anchor_a -> anchor_b)
        n = delta / dist

        # 각 바디 중심에서 앵커까지의 벡터 (토크 계산용)
        r_a = pa - self.body_a.center
        r_b = pb - self.body_b.center

        # 수직 벡터 (2D에서 r⊥ * ω = 각속도에 의한 속도 성분)
        r_a_perp = Vector2D(-r_a.y, r_a.x)
        r_b_perp = Vector2D(-r_b.y, r_b.x)

        # 앵커에서의 속도 (선형 + 각속도 성분)
        v_a = self.body_a.velocity + r_a_perp * self.body_a.angular_velocity
        v_b = self.body_b.velocity + r_b_perp * self.body_b.angular_velocity

        # 제약 축 방향 상대 속도
        relative_velocity = v_b - v_a
        rel_v_n = relative_velocity.dot(n)

        # 역질량 / 역관성
        inv_mass_a = 0.0 if self.body_a.is_static else 1.0 / self.body_a.mass
        inv_mass_b = 0.0 if self.body_b.is_static else 1.0 / self.body_b.mass
        inv_inertia_a = 0.0 if self.body_a.is_static else 1.0 / self.body_a.inertia
        inv_inertia_b = 0.0 if self.body_b.is_static else 1.0 / self.body_b.inertia

        # 제약 방향에 대한 유효 질량(Effective mass)
        k = (
            inv_mass_a
            + inv_mass_b
            + (r_a_perp.dot(n) ** 2) * inv_inertia_a
            + (r_b_perp.dot(n) ** 2) * inv_inertia_b
        )

        if k == 0:
            return

        # Baumgarte 스타일의 bias term:
        # diff(위치 오차)를 속도 수준에서 조금씩 줄여나가도록 만듦
        # stiffness를 이 오차를 얼마나 급진적으로 줄일지에 대한 계수로 사용
        beta = self.stiffness
        bias = beta * diff  # dt를 안 쓰는 대신 stiffness에 흡수했다고 생각하면 됨

        # 제약 조건: rel_v_n + bias ≈ 0 이 되도록 임펄스를 계산
        j = -(rel_v_n + bias) / k

        # 제약 축 방향 임펄스
        impulse = n * j

        # 두 바디에 반대 방향으로 임펄스 적용
        if not self.body_a.is_static:
            self.body_a.apply_impulse(-impulse, r_a)
        if not self.body_b.is_static:
            self.body_b.apply_impulse( impulse, r_b)
