class Node:
    def __init__(self, index: int, id: str, x: float, y: float):
        self.index = index
        self.id = id
        self.x = x
        self.y = y
        self.edges = []

    def __str__(self) -> str:
        return self.id
