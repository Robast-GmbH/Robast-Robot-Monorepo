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
