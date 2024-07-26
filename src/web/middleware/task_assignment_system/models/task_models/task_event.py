from dataclasses import dataclass
from typing import Any


@dataclass
class TaskEvent:
    category: str
    description: dict[str, Any]

    def to_json(self) -> dict[str, Any]:
        return {"category": self.category, "description": self.description}


class DropOff(TaskEvent):
    def __init__(
        self,
        target_id: str,
        items_by_change: dict[str, int],
        module_id: int,
        drawer_id: int,
        assignee_name: str,
    ):
        super().__init__(
            category="dropoff",
            description={
                "place": target_id,
                "handler": "drawer_ingestor",
                "payload": {
                    "sku": f"{module_id},{drawer_id},{items_by_change},{assignee_name}",
                    "quantity": 1,
                },
            },
        )


class PickUp(TaskEvent):
    def __init__(
        self,
        target_id: str,
        items_by_change: dict[str, int],
        module_id: int,
        drawer_id: int,
        assignee_name: str,
    ):
        super().__init__(
            category="pickup",
            description={
                "place": target_id,
                "handler": "drawer_dispenser",
                "payload": {
                    "sku": f"{module_id},{drawer_id},{items_by_change},{assignee_name}",
                    "quantity": 1,
                },
            },
        )


class GoToPlace(TaskEvent):
    def __init__(self, target_id: str):
        super().__init__(
            category="go_to_place",
            description={"place": target_id},
        )
