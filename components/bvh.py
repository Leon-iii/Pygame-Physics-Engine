from components.body import Body

class AABB:
    def __init__(self, min_x: float, min_y: float, max_x: float, max_y: float):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

    @classmethod # self 대신 cls 사용.
    def from_body(cls, body: Body) -> "AABB": # "AABB" - 전방 참조
        min_x, min_y, max_x, max_y = body.compute_aabb()
        return cls(min_x, min_y, max_x, max_y)

    def intersects(self, other: "AABB") -> bool:
        # 분리축: x 또는 y 방향으로 완전히 떨어져 있으면 겹치지 않음
        if self.max_x < other.min_x or other.max_x < self.min_x:
            return False
        if self.max_y < other.min_y or other.max_y < self.min_y:
            return False
        return True

    @staticmethod
    def merge(a: "AABB", b: "AABB") -> "AABB":
        return AABB(
            min(a.min_x, b.min_x),
            min(a.min_y, b.min_y),
            max(a.max_x, b.max_x),
            max(a.max_y, b.max_y),
        )
    
class BVHNode:
    """BVH - Boundary Volume Hierarchy의 트리의 노드."""
    def __init__(self,
                 aabb: AABB,
                 left: "BVHNode | None" = None,
                 right: "BVHNode | None" = None,
                 body: Body | None = None):
        self.aabb = aabb
        self.left = left
        self.right = right
        self.body = body  # 리프면 Body, 내부노드는 None

    def is_leaf(self) -> bool:
        return self.body is not None

def build_bvh(bodies: list[Body]) -> BVHNode | None:
    """강체들의 집합으로부터 BVH를 빌드. (매 프레임 호출)"""
    if not bodies:
        return None

    # 리프: 바디가 하나뿐이면 바로 리프 노드 생성
    if len(bodies) == 1:
        aabb = AABB.from_body(bodies[0])
        return BVHNode(aabb=aabb, body=bodies[0])

    # 1) 각 Body의 AABB와 중심 x값을 계산
    body_aabbs = [(body, AABB.from_body(body)) for body in bodies]
    body_aabbs.sort(key=lambda pair: (pair[1].min_x + pair[1].max_x) * 0.5)

    mid = len(body_aabbs) // 2
    left_bodies = [b for (b, _) in body_aabbs[:mid]]
    right_bodies = [b for (b, _) in body_aabbs[mid:]]

    left_node = build_bvh(left_bodies)
    right_node = build_bvh(right_bodies)

    # 2) 자식 노드 둘의 AABB를 merge해서 부모 AABB 생성
    merged_aabb = AABB.merge(left_node.aabb, right_node.aabb)

    return BVHNode(aabb=merged_aabb, left=left_node, right=right_node)

def collect_pairs(root: BVHNode | None) -> list[tuple[Body, Body]]:
    pairs: list[tuple[Body, Body]] = []
    if root is None:
        return pairs

    def is_leaf(node: BVHNode) -> bool:
        return node.body is not None

    def self_query(a: BVHNode | None, b: BVHNode | None):
        if a is None or b is None:
            return

        # AABB가 겹치지 않으면 이 조합 전체 스킵
        if not a.aabb.intersects(b.aabb):
            return

        # 둘 다 리프: Body 쌍 출력
        if is_leaf(a) and is_leaf(b):
            if a.body is not b.body:
                pairs.append((a.body, b.body))
            return

        # 한쪽만 리프인 경우: 리프 vs (다른 쪽 서브트리)
        if is_leaf(a) and not is_leaf(b):
            self_query(a, b.left)
            self_query(a, b.right)
            return

        if is_leaf(b) and not is_leaf(a):
            self_query(a.left, b)
            self_query(a.right, b)
            return

        # 둘 다 내부 노드: 자식들 조합 전부 검사
        self_query(a.left,  b.left)
        self_query(a.left,  b.right)
        self_query(a.right, b.left)
        self_query(a.right, b.right)

    def traverse(node: BVHNode | None):
        """각 노드에 대해 왼쪽/오른쪽 서브트리 쌍을 self_query하고 재귀"""
        if node is None or is_leaf(node):
            return

        # 이 노드의 왼쪽/오른쪽 서브트리 사이의 충돌 후보
        self_query(node.left, node.right)

        # 각 자식 서브트리 내부 재귀적으로 처리
        traverse(node.left)
        traverse(node.right)

    traverse(root)
    return pairs



################################# 디버그: BVH 화면에 그리기 ####################################


import pygame
from typing import Optional

def draw_bvh_node(
    surface: "pygame.Surface",
    node: BVHNode,
    depth: int = 0,
    max_depth: Optional[int] = None,
):
    """BVH 트리를 재귀적으로 순회하면서 각 노드의 AABB를 화면에 그린다."""
    if node is None:
        return

    if max_depth is not None and depth > max_depth:
        return

    aabb = node.aabb
    # AABB를 Pygame Rect로 변환 (월드 좌표 == 스크린 좌표 기준)
    
    w = aabb.max_x - aabb.min_x
    h = aabb.max_y - aabb.min_y

    x = aabb.min_x
    y = pygame.display.get_window_size()[1] - aabb.min_y - h

    # 깊어질수록 색이 어두워지게
    base_color = (0, 255, 0)
    g = max(0, base_color[1] - depth * 40)
    color = (base_color[0], g, base_color[2])

    # 두께 1픽셀로 라인 그리기
    pygame.draw.rect(surface, color, pygame.Rect(x, y, w, h), width=1)

    # 자식들도 재귀적으로 그리기
    if node.left:
        draw_bvh_node(surface, node.left, depth + 1, max_depth)
    if node.right:
        draw_bvh_node(surface, node.right, depth + 1, max_depth)