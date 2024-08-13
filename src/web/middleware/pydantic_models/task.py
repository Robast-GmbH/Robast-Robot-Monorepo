import json
from pydantic import BaseModel
from typing import Any
from pydantic_models.sub_task import SubTask


class Task(BaseModel):
    id: str
    name: str
    status: str
    assignee_name: str
    requirements: dict[str, Any]
    subtasks: list[SubTask]
    is_monolithic: bool
    earliest_start_time: int
    priority: int

    @classmethod
    def from_database_row(cls, row: list[Any], subtasks: list[SubTask]) -> "Task":
        return cls(
            id=row[0],
            name=row[1],
            status=row[2],
            assignee_name=row[3],
            requirements=json.loads(row[4]),
            subtasks=subtasks,
            is_monolithic=row[6],
            earliest_start_time=row[7],
            priority=row[8],
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "status": self.status,
            "assignee_name": self.assignee_name,
            "requirements": self.requirements,
            "subtasks": [subtask.to_json() for subtask in self.subtasks],
            "is_monolithic": self.is_monolithic,
            "earliest_start_time": self.earliest_start_time,
            "priority": self.priority,
        }
