from task_assignment_system.models.node import Node
from dataclasses import dataclass, field


@dataclass
class Edge:
    node1: Node
    node2: Node
    weight: float = field(init=False)

    def __post_init__(self) -> None:
        self.weight = self.__euclidean_distance(self.node1, self.node2)

    @staticmethod
    def __euclidean_distance(node1: Node, node2: Node) -> float:
        return ((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2) ** 0.5
