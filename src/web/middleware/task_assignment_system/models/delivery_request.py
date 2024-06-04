from task_assignment_system.models.node import Node


class DeliveryRequest:
    def __init__(
        self, required_drawer_type: str, target: Node, start: Node = None
    ) -> None:
        self.required_drawer_type = required_drawer_type
        self.target = target
        self.start = start
