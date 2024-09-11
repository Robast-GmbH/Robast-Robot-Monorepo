import json
import time
from sqlalchemy import Boolean, Column, Integer, String, JSON, ForeignKey
from sqlalchemy.orm import relationship
from db_models.task import Base
from pydantic_models.action import Action
from pydantic_models.submodule_address import SubmoduleAddress
from typing import Dict, Any


class Subtask(Base):
    __tablename__ = "subtasks"

    id = Column(String, primary_key=True)
    name = Column(String)
    status = Column(String)
    assignee_name = Column(String)
    parent_id = Column(String, ForeignKey("tasks.id"))
    requires_task_id = Column(String)
    is_part_of_monolith = Column(Boolean)
    target_id = Column(String)
    priority = Column(Integer)
    earliest_start_time = Column(Integer)
    requirements: Dict[str, Any] = Column(JSON)
    action: Dict[str, Any] = Column(JSON)

    parent = relationship("Task", back_populates="subtasks")

    def to_json(self):
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
            "requirements": self.requirements if self.requirements else {},
            "action": self.action,
        }

    def contains_submodule_process_action(self) -> bool:
        action = self.action
        while action:
            if action["name"] == "submodule_process":
                return True
            action = action["subaction"]
        return False

    def write_submodule_address(self, submodule_address: SubmoduleAddress) -> None:
        submodule_address_json = submodule_address.to_json()
        self.requirements["submodule_address"] = submodule_address_json
        action = self.action
        while action:
            if action["name"] == "submodule_process":
                action["parameters"]["submodule_address"] = submodule_address_json
            action = action["subaction"]

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
                    "sku": json.dumps(self.action),
                    "quantity": 1,
                },
            },
        }
