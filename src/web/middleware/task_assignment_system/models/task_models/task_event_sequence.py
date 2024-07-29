from dataclasses import dataclass
from typing import Any
from task_assignment_system.models.task_models.task_event import TaskEvent


@dataclass
class TaskEventSequence:
    events: list[TaskEvent]

    def to_json(self) -> dict[str, Any]:
        return {
            "category": "sequence",
            "description": {
                "activities": [event.to_json() for event in self.events],
            },
        }
