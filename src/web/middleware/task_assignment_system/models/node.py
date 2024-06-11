from task_assignment_system.models.edge import Edge
from dataclasses import dataclass


@dataclass
class Node:
    index: int
    id: str
    x: float
    y: float
    edges: list[Edge] = []

    def __str__(self) -> str:
        return self.id
