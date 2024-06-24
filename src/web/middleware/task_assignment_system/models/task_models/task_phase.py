from dataclasses import dataclass
from typing import Any
from task_assignment_system.models.task_models.task_event_sequence import (
    TaskEventSequence,
)
from task_assignment_system.models.task_models.task_event import PickUp, DropOff


@dataclass
class TaskPhase:
    event_sequence: TaskEventSequence

    def to_json(self) -> dict[str, Any]:
        return {"activity": self.event_sequence.to_json()}

    @classmethod
    def create_pickup_phase(
        cls,
        target_id: str,
        payload: dict[str, int],
        module_id: int,
        drawer_id: int,
        assignee_name: str,
    ) -> "TaskPhase":
        return cls(
            TaskEventSequence(
                [PickUp(target_id, payload, module_id, drawer_id, assignee_name)]
            )
        )

    @classmethod
    def create_dropoff_phase(
        cls,
        target_id: str,
        payload: dict[str, int],
        module_id: int,
        drawer_id: int,
        assignee_name: str,
    ) -> "TaskPhase":
        return cls(
            TaskEventSequence(
                [DropOff(target_id, payload, module_id, drawer_id, assignee_name)]
            )
        )
