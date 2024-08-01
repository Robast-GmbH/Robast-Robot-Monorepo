from pydantic import BaseModel
from typing import Optional, Any


class Action(BaseModel):
    name: str
    status: str
    parameters: dict[str, Any]
    subaction: Optional["Action"]

    def to_json(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "parameters": self.parameters,
            "subaction": self.subaction.to_json() if self.subaction else None,
        }
