from task_assignment_system.models.node import Node
from dataclasses import dataclass


@dataclass
class DeliveryRequest:
    required_drawer_type: int
    target: Node
    start: Node = None
