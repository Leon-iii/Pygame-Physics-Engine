import math
from abc import ABC, abstractmethod
from components.vector import Vector2D


# 모든 물리 객체(질량, 속도, 각속도 등)의 공통 속성을 담는 기본 Body 클래스
# ABC - Abstract Base Class. 추상 베이스 클래스 - 인스턴스 생성 불가
class Body(ABC):
    def __init__(self, x, y,
                 mass=1, bounce=0.5,
                 name=None,
                 is_static=False,ignore_gravity=False,
                 color=None,
                 linear_damp=None, angular_damp=None,
                 is_breakable=None, break_impulse=None,
                 fracture_gen: int = 0):
        # 물체의 중심 위치
        self.center = Vector2D(x, y)
        # 물체의 회전 각도 (라디안)
        self.angle = 0
        # 디버깅/식별용 이름
        self.name = name

        # 선형 속도, 각속도
        self.velocity = Vector2D(0, 0)
        self.angular_velocity = 0

        # 가속도, 각가속도는 저장 X. a = F/m으로 매 순간 구해서 쓰면 된다.

        # 관성 모멘트 (도형별로 계산)
        self.inertia = None
        # 정적 물체는 질량과 관성을 무한대로 취급해서 힘에 반응하지 않게 함
        self.mass = mass if not is_static else float("inf")
        # 반발 계수 (0 = 완전 비탄성, 1 = 완전 탄성)
        self.bounce = bounce
        # 움직이지 않는 지형/벽인지 여부
        self.is_static = is_static

        self.ignore_gravity = ignore_gravity
        self.color = color

        self.linear_damp = linear_damp
        self.angular_damp = angular_damp

        self.is_breakable = is_breakable
        self.break_impulse = break_impulse
        self.fracture_gen = fracture_gen

    def apply_impulse(self, impulse: Vector2D, r: Vector2D) -> None:
        """접촉점 offset r에서 임펄스 impulse를 적용해서 선형/각속도를 갱신"""
        if self.is_static:
            return

        # 선형 속도 변화
        self.velocity += impulse / self.mass

        # 각속도 변화 (2D에서 토크 = r x J)
        self.angular_velocity += r.cross(impulse) / self.inertia

    def teleport(self, coord: Vector2D, maintain_momentum:bool = False) -> None:
        """Body를 순간이동 시킴. """

        self.center = coord

        if not maintain_momentum:
            # 관성 삭제
            self.velocity = Vector2D()
            self.angular_velocity = 0

    @abstractmethod # 추상 메서드
    def get_vertices(self) -> list[tuple[int, int]]:
        """월드 좌표계에서의 꼭짓점 목록 반환"""
        pass

    @abstractmethod # 추상 메서드
    def compute_aabb(self) -> tuple[Vector2D, Vector2D]:
        """이 Body를 감싸는 AABB(또는 적절한 바운딩 볼륨)를 계산해서 반환"""
        pass
        


# 축 정렬이 아닌 회전 가능한 직사각형(사각형)을 표현하는 클래스
class Rectangle(Body):
    def __init__(self, x, y,
                width, height,
                mass=1, bounce=0.5,
                name=None,
                is_static=False, ignore_gravity=False,
                color: tuple[int, int, int] | None = None,
                linear_damp: float | None = None,
                angular_damp: float | None = None,
                is_breakable: bool = False,
                break_impulse: float = float("inf"),
                fracture_gen: int = 0
                ):
        super().__init__(x, y, mass, bounce, name, is_static, ignore_gravity, color, linear_damp, angular_damp, is_breakable, break_impulse, fracture_gen)
        self.width = width
        self.height = height

        # 중심(0,0)을 기준으로 한 로컬 좌표계에서의 꼭짓점들
        half_width = self.width / 2
        half_height = self.height / 2
        self.local_vertices = [
            Vector2D(-half_width, -half_height),
            Vector2D(half_width, -half_height),
            Vector2D(half_width, half_height),
            Vector2D(-half_width, half_height)
        ]

        # 직사각형에 대한 관성 모멘트 (I = 1/12 m (w² + h²)), 정적 물체는 무한대
        self.inertia = (1 / 12) * mass * (width * width + height * height) if not is_static else float("inf")

    # 현재 회전 각도에 따른 x, y 축(직사각형의 로컬 축)을 반환
    def get_axes(self) -> list[Vector2D]:
        # x축: 각도 방향, y축: 그에 수직한 방향
        self.x_axis = Vector2D(math.cos(self.angle), math.sin(self.angle))
        self.y_axis = Vector2D(-math.sin(self.angle), math.cos(self.angle))

        return [self.x_axis, self.y_axis]

    # 월드 좌표계에서의 꼭짓점 목록 반환 (로컬 정점 → 회전 → 중심 위치 더하기)
    def get_vertices(self) -> list[tuple[int, int]]:
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]

    # 사각형 회전 (기존 각도에 추가)
    def rotate(self, angle, in_radians=True) -> None:
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

    # 추상메서드 구현
    def compute_aabb(self) -> tuple[float, float, float, float]:
        """이 Body를 감싸는 AABB(또는 적절한 바운딩 볼륨)를 계산해서 반환"""
        
        min_x = float("inf")
        min_y = float("inf")

        max_x = -float("inf")
        max_y = -float("inf")

        for vertex in self.get_vertices():

            min_x = min(min_x, vertex[0])
            min_y = min(min_y, vertex[1])

            max_x = max(max_x, vertex[0])
            max_y = max(max_y, vertex[1])

        return (min_x, min_y, max_x, max_y)


# 임의의 볼록/오목 다각형을 표현하는 클래스
class Polygon(Body):
    def __init__(self, x, y,
                vertices: list[Vector2D, list, tuple],
                mass=1, bounce=0.5,
                name=None,
                is_static=False, ignore_gravity=False, 
                color: tuple[int, int, int] | None = None,
                linear_damp: float | None = None,
                angular_damp: float | None = None,
                is_breakable: bool = False,
                break_impulse: float = float("inf"),
                fracture_gen: int = 0
                ):
        super().__init__(x, y, mass, bounce, name, is_static, ignore_gravity, color, linear_damp, angular_damp, is_breakable, break_impulse, fracture_gen)

        # 주어진 정점들의 기하학적 중심(평균 위치)을 구함
        centroid = (
            sum(vertex[0] for vertex in vertices) / len(vertices),
            sum(vertex[1] for vertex in vertices) / len(vertices),
        )

        # 다각형의 중심을 (0,0)으로 옮긴 로컬 정점 목록 생성
        self.local_vertices = [Vector2D(vertex[0] - centroid[0], vertex[1] - centroid[1]) for vertex in vertices]

        # 다각형의 관성 모멘트 계산, 정적 물체는 무한대
        self.inertia = self.calculate_inertia() if not is_static else float("inf")

    # 월드 좌표계에서의 꼭짓점 목록 반환 (로컬 정점 → 회전 → 중심 위치 더하기)
    def get_vertices(self) -> list[tuple[int, int]]:
        return [vertex.rotate(self.angle).add(self.center) for vertex in self.local_vertices]

    # 다각형의 질량 중심 기준 관성 모멘트를 계산하는 함수
    #   1) 다각형을 여러 삼각형으로 분할 (연속된 두 정점과 원점으로 이루어진 삼각형)
    #   2) 각 삼각형의 넓이/질량 중심/관성 모멘트를 합산
    #   3) 전체 질량 중심을 구한 뒤, 평행축 정리를 통해 최종 관성 모멘트로 변환
    def calculate_inertia(self) -> float:
        area = 0          # 다각형 누적 넓이
        center = Vector2D(0, 0)  # 누적 질량 중심 (점진적으로 업데이트)
        mmoi = 0          # 원점 기준 관성 모멘트(moment of inertia, second moment of area)

        prev = len(self.local_vertices) - 1
        for index in range(len(self.local_vertices)):
            a = self.local_vertices[prev]
            b = self.local_vertices[index]

            # 삼각형 (0, a, b)의 부호 있는 넓이 (외적의 절반)
            area_step = a.cross(b) / 2

            # 해당 삼각형의 질량 중심 (정점 3개의 평균, 여기서는 0, a, b → (a+b)/3)
            center_step = (a + b) / 3

            # 삼각형의 원점 기준 2차 모멘트(공식 이용)
            mmoi_step = area_step * (a.dot(a) + b.dot(b) + a.dot(b)) / 6

            # 전체 질량 중심을 가중 평균으로 업데이트
            center = (center * area + center_step * area_step) / (area + area_step)
            area += area_step
            mmoi += mmoi_step

            prev = index

        # 면적에 맞게 밀도(질량/넓이)를 곱해서 실제 관성 모멘트로 변환
        density = self.mass / area
        mmoi *= density

        # 현재 mmoi는 원점 기준이므로, 평행축 정리로 질량 중심 기준 관성 모멘트로 변환
        mmoi -= self.mass * center.dot(center)

        return mmoi

    # 다각형 회전 (기존 각도에 추가)
    def rotate(self, angle, in_radians=True) -> None:
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle

    # 추상메서드 구현
    def compute_aabb(self) -> tuple[float, float, float, float]:
        """이 Body를 감싸는 AABB(또는 적절한 바운딩 볼륨)를 계산해서 반환"""
        
        min_x = float("inf")
        min_y = float("inf")

        max_x = -float("inf")
        max_y = -float("inf")

        for vertex in self.get_vertices():

            min_x = min(min_x, vertex[0])
            min_y = min(min_y, vertex[1])

            max_x = max(max_x, vertex[0])
            max_y = max(max_y, vertex[1])

        return (min_x, min_y, max_x, max_y)


# 원형(원판)을 표현하는 클래스
class Circle(Body):
    def __init__(self, x, y,
                radius,
                mass=1, bounce=0.5,
                name=None, 
                is_static=False, ignore_gravity=False,
                color: tuple[int, int, int] | None = None,
                linear_damp: float | None = None,
                angular_damp: float | None = None,
                is_breakable: bool = False,
                break_impulse: float = float("inf"),
                fracture_gen: int = 0
                ):
        super().__init__(x, y, mass, bounce, name, is_static, ignore_gravity, color, linear_damp, angular_damp, is_breakable, break_impulse, fracture_gen)
        # 원판의 관성 모멘트 (I = 1/2 m r²), 정적 물체는 무한대
        self.radius = radius
        self.inertia = (1 / 2) * mass * radius * radius if not is_static else float("inf")

    # 원도 회전 (각도만 업데이트, 실제 충돌에는 영향 없음)
    def rotate(self, angle, in_radians=True) -> None:
        if not in_radians:
            angle = math.radians(angle)
        self.angle += angle


    # 월드 좌표계에서의 꼭짓점 목록 반환 (원의 경우 36각형으로 근사)
    def get_vertices(self) -> list[tuple[int, int]]:
        return [Vector2D(self.radius, 0).rotate(theta, in_radians=False).add(self.center) for theta in range(0, 360, 10)]

    # 추상메서드 구현
    def compute_aabb(self) -> tuple[float, float, float, float]:
        """이 Body를 감싸는 AABB(또는 적절한 바운딩 볼륨)를 계산해서 반환"""

        return (self.center.x - self.radius, self.center.y - self.radius,
                self.center.x + self.radius, self.center.y + self.radius)
    

def polygon_area(vertices: list[Vector2D]) -> float:
    """가우스 면적 공식 사용 - 다각형 면적 계산"""
    area = 0.0
    n = len(vertices)
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i].cross(vertices[j])
    return abs(area) * 0.5