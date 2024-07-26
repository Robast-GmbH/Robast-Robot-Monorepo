from __future__ import annotations
from task_assignment_system.models.node import Node
from task_assignment_system.models.edge import Edge
from dataclasses import dataclass, field
from typing import Any
import heapq


@dataclass
class NavGraph:
    nodes: list[Node]
    min_distances: dict[Node, dict[Node, float]] = field(init=False)

    def __post_init__(self) -> None:
        # Minimal distances between all nodes, accessed by min_distances[node1][node2]
        self.min_distances = {}
        for node in self.nodes:
            self.min_distances[node] = self.__dijkstra(self.nodes, node)

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> NavGraph:
        # Extract the vertices and edges
        vertices = data["vertices"]
        edges = data["edges"]
        nodes = []

        for index, vertex in enumerate(vertices):
            nodes.append(Node(index, vertex["name"], vertex["x"], vertex["y"]))

        for edge in edges:
            nodes[edge["v1_idx"]].edges.append(
                Edge(nodes[edge["v1_idx"]], nodes[edge["v2_idx"]])
            )
            nodes[edge["v2_idx"]].edges.append(
                Edge(nodes[edge["v2_idx"]], nodes[edge["v1_idx"]])
            )
        return cls(nodes)

    def get_node_by_id(self, id: str) -> Node:
        for node in self.nodes:
            if node.id == id:
                return node
        raise ValueError(f"Node with id {id} not found.")

    def __dijkstra(self, graph: list[Node], start: Node) -> dict[Node, float]:
        # Initialize the distance dictionary with infinite distances for all nodes except the start node
        distances = {node: float("infinity") for node in graph}
        distances[start] = 0

        # Initialize the priority queue with the start node
        queue = [(0, start)]

        while queue:
            # Get the node with the smallest distance
            current_distance, current_node = heapq.heappop(queue)

            # If the current distance is greater than the recorded distance for the current node, skip it
            if current_distance > distances[current_node]:
                continue

            # Check all neighbors of the current node
            for edge in graph[current_node.index].edges:
                distance = current_distance + edge.weight

                # If the calculated distance is less than the recorded distance for the neighbor, update it
                if distance < distances[edge.node2]:
                    distances[edge.node2] = distance
                    heapq.heappush(queue, (distance, edge.node2))

        return distances
