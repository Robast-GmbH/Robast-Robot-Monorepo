from dataclasses import dataclass, field


@dataclass
class Node:
    index: int
    id: str
    x: float
    y: float
    edges: list = field(default_factory=list)

    def __str__(self) -> str:
        return self.id

    def __hash__(self) -> int:
        return hash(self.id)
