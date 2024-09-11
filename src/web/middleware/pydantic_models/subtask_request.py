from pydantic import BaseModel
from typing import Any
from pydantic_models.action import Action
from db_models.subtask import Subtask


class SubtaskRequest(BaseModel):
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

    def to_db_subtask(self) -> Subtask:
        return Subtask(
            id=self.id,
            name=self.name,
            status=self.status,
            assignee_name=self.assignee_name,
            parent_id=self.parent_id,
            requires_task_id=self.requires_task_id,
            is_part_of_monolith=self.is_part_of_monolith,
            target_id=self.target_id,
            priority=self.priority,
            earliest_start_time=self.earliest_start_time,
            requirements=self.requirements,
            action=self.action.to_json(),
        )
