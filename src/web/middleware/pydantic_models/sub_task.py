import json
import time
from pydantic import BaseModel
from typing import Any
from pydantic_models.action import Action
from pydantic_models.submodule_address import SubmoduleAddress


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

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "SubTask":
        return cls(
            id=json_data["id"],
            name=json_data["name"],
            status=json_data["status"],
            assignee_name=json_data["assignee_name"],
            parent_id=json_data["parent_id"],
            requires_task_id=json_data["requires_task_id"],
            is_part_of_monolith=json_data["is_part_of_monolith"],
            target_id=json_data["target_id"],
            priority=json_data["priority"],
            earliest_start_time=json_data["earliest_start_time"],
            requirements=json_data["requirements"],
            action=Action.from_json(json_data["action"]),
        )

    @classmethod
    def from_database_row(cls, row: list[Any]) -> "SubTask":
        return cls(
            id=row[0],
            name=row[1],
            status=row[2],
            assignee_name=row[3],
            parent_id=row[4],
            requires_task_id=row[5],
            is_part_of_monolith=row[6],
            target_id=row[7],
            priority=row[8],
            earliest_start_time=row[9],
            requirements=json.loads(row[10]),
            action=Action.from_json(json.loads(row[11])),
        )

    def contains_submodule_process_action(self) -> bool:
        action = self.action
        while action:
            if action.name == "submodule_process":
                return True
            action = action.subaction
        return False

    def write_submodule_address(self, submodule_address: SubmoduleAddress) -> None:
        submodule_address_json = submodule_address.to_json()
        self.requirements["submodule_address"] = submodule_address_json
        action = self.action
        while action:
            if action.name == "submodule_process":
                action.parameters["submodule_address"] = submodule_address_json
            action = action.subaction

    def to_json(self) -> dict[str, Any]:
        return {
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
                "requester": "task_system",
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
                "handler": "submodule_dispenser",
                "payload": {
                    "sku": json.dumps(self.action.to_json()),
                    "quantity": 1,
                },
            },
        }
