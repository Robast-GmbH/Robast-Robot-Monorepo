from task_assignment_system.models.node import Node
from task_assignment_system.models.drawer import Drawer
from dataclasses import dataclass
from typing import Any
import time


@dataclass
class Task:
    id: str
    assignee_name: str
    task_type: str
    requires_task_id: str | None
    drawer: Drawer
    target: Node

    def to_json(self) -> dict[str, Any]:
        return {
            "type": "robot_task_request",
            "robot": "rb_theron",
            "fleet": "deliveryRobot",
            "request": {
                "unix_millis_earliest_start_time": int(time.time() * 1000),
                "unix_millis_request_time": int(time.time() * 1000),
                "priority": {"value": 0, "type": "binary"},
                "category": "compose",
                "description": {
                    "category": "custom_action",
                    "phases": [
                        {
                            "activity": {
                                "category": "sequence",
                                "description": {
                                    "activities": [
                                        {
                                            "category": self.task_type,
                                            "description": {
                                                "place": self.target.id,
                                                "handler": (
                                                    "drawer_ingestor"
                                                    if self.task_type == "dropoff"
                                                    else "drawer_dispenser"
                                                ),
                                                "payload": {
                                                    "sku": f"{self.drawer.id},some_stuff,{self.assignee_name}",
                                                    "quantity": 1,
                                                },
                                            },
                                        },
                                    ]
                                },
                            }
                        }
                    ],
                },
                "requester": "task_assignment_system",
            },
        }

    def __str__(self) -> str:
        return f"Task {self.id} - {self.task_type} - {self.drawer} - {self.target}"
