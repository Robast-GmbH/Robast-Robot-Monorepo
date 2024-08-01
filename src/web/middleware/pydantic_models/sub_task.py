import json
import time
from pydantic import BaseModel
from typing import Any
from pydantic_models.action import Action


class SubTask(BaseModel):
    id: str
    name: str
    status: str
    assignee_name: str
    parent_id: str
    requires_task_id: str | None
    is_part_of_monolith: bool
    target_id: str
    priority: int
    earliest_start_time: int
    requirements: dict[str, Any]
    action: Action

    def to_json(self) -> dict[str, Any]:
        return {
            "task": {
                "id": self.id,
                "name": self.name,
                "status": self.status,
                "assignee_name": self.assignee_name,
                "parent_id": self.parent_id,
                "requires_task_id": self.requires_task_id,
                "is_part_of_monolith": self.is_part_of_monolith,
                "target_id": self.target_id,
                "priority": self.priority,
                "earliest_start_time": self.earliest_start_time,
                "requirements": self.requirements,
                "action": self.action.to_json(),
            }
        }

    def to_robot_task_request(self) -> dict[str, Any]:
        return {
            "type": "robot_task_request",
            "robot": self.assignee_name,
            "fleet": "deliveryRobot",
            "request": {
                "unix_millis_earliest_start_time": int(time.time() * 1000),
                "unix_millis_request_time": int(time.time() * 1000),
                "priority": {"value": 0, "type": "binary"},
                "category": "compose",
                "description": {
                    "category": "custom_action",
                    "phases": [self.__create_task_phase()],
                },
                "requester": "task_assignment_system",
            },
        }

    def __create_task_phase(self) -> dict[str, Any]:
        return {"activity": self.__create_task_sequence()}

    def __create_task_sequence(self) -> dict[str, Any]:
        return {
            "category": "sequence",
            "description": {
                "activities": [self.__create_activity()],
            },
        }

    def __create_activity(self) -> dict[str, Any]:
        return {
            "category": "pickup",
            "description": {
                "place": self.target_id,
                "handler": "drawer_dispenser",
                "payload": {
                    "sku": json.dumps(self.action.to_json()),
                    "quantity": 1,
                },
            },
        }
