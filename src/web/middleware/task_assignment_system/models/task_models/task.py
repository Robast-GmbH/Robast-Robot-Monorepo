from dataclasses import dataclass
from typing import Any
import time
import uuid
from task_assignment_system.models.task_models.task_phase import TaskPhase
from pydantic_models.delivery_request import DeliveryRequest
from pydantic_models.drawer import Drawer


@dataclass
class Task:
    # unique ID
    id: str
    # pickup, dropoff or goto
    task_type: str

    status: str
    items_by_change: dict[str, int]
    creation_date: int
    requires_task_id: str | None
    assignee_name: str
    module_id: int
    drawer_id: int
    drawer_type: int
    priority: int
    target_id: str
    auth_users: list[str]
    auth_user_groups: list[str]
    task_phases: list[TaskPhase]

    @classmethod
    def from_delivery_request(
        cls,
        assignee_name: str,
        delivery_request: DeliveryRequest,
        drawer: Drawer,
    ) -> list["Task"]:
        tasks = []
        pick_up_id = str(uuid.uuid4())
        if delivery_request.start_id is not None:
            pick_up_task_phase = TaskPhase.create_pickup_phase(
                delivery_request.start_id,
                delivery_request.items_by_change,
                drawer.address.module_id,
                drawer.address.drawer_id,
                assignee_name,
            )
            pick_up_task = Task(
                pick_up_id,
                "pickup",
                "pending",
                delivery_request.items_by_change,
                int(time.time()),
                None,
                assignee_name,
                drawer.address.module_id,
                drawer.address.drawer_id,
                drawer.size,
                0,
                delivery_request.start_id,
                delivery_request.sender_user_ids,
                delivery_request.sender_user_groups,
                [pick_up_task_phase],
            )
            tasks.append(pick_up_task)

        if delivery_request.target_id is not None:
            drop_off_items_by_change = {
                key: -value for key, value in delivery_request.items_by_change.items()
            }
            drop_off_task_phase = TaskPhase.create_dropoff_phase(
                delivery_request.target_id,
                drop_off_items_by_change,
                drawer.address.module_id,
                drawer.address.drawer_id,
                assignee_name,
            )

            drop_off_task = Task(
                str(uuid.uuid4()),
                "dropoff",
                "pending",
                drop_off_items_by_change,
                int(time.time()),
                pick_up_id if delivery_request.start_id else None,
                assignee_name,
                drawer.address.module_id,
                drawer.address.drawer_id,
                drawer.size,
                0,
                delivery_request.target_id,
                delivery_request.recipient_user_ids,
                delivery_request.recipient_user_groups,
                [drop_off_task_phase],
            )
            tasks.append(drop_off_task)

        return tasks

    def to_robot_task_request(self) -> dict[str, Any]:
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
                    "phases": [phase.to_json() for phase in self.task_phases],
                },
                "requester": "task_assignment_system",
            },
        }

    def to_json(self) -> dict[str, Any]:
        return {
            "task": {
                "id": self.id,
                "task_type": self.task_type,
                "status": self.status,
                "items_by_change": self.items_by_change,
                "creation_date": self.creation_date,
                "requires_task_id": self.requires_task_id,
                "assignee_name": self.assignee_name,
                "module_id": self.module_id,
                "drawer_id": self.drawer_id,
                "drawer_type": self.drawer_type,
                "priority": self.priority,
                "target_id": self.target_id,
                "auth_users": self.auth_users,
                "auth_user_groups": self.auth_user_groups,
                "task_phases": [phase.to_json() for phase in self.task_phases],
            }
        }

    def __str__(self) -> str:
        return f"Task {self.id} - {self.task_type} - {self.module_id}_{self.drawer_id} - {self.target_id}"
