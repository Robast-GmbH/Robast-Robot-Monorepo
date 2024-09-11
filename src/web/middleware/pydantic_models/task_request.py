from pydantic import BaseModel
from typing import Any
from pydantic_models.subtask_request import SubtaskRequest
from db_models.task import Task


class TaskRequest(BaseModel):
    id: str
    name: str
    status: str
    assignee_name: str
    requirements: dict[str, Any]
    subtasks: list[SubtaskRequest]
    is_monolithic: bool
    earliest_start_time: int
    priority: int

    def to_db_task(self) -> Task:
        return Task(
            id=self.id,
            name=self.name,
            status=self.status,
            assignee_name=self.assignee_name,
            requirements=self.requirements,
            is_monolithic=self.is_monolithic,
            earliest_start_time=self.earliest_start_time,
            priority=self.priority,
            subtasks=[subtask.to_db_subtask() for subtask in self.subtasks],
        )
