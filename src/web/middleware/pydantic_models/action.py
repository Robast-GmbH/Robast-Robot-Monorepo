from pydantic import BaseModel
from typing import Optional, Any


class Action(BaseModel):
    id: str
    name: str
    status: str
    parameters: dict[str, Any]
    subaction: Optional["Action"]

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "Action":
        return cls(
            id=json_data["id"],
            name=json_data["name"],
            status=json_data["status"],
            parameters=json_data["parameters"],
            subaction=(
                Action.from_json(json_data["subaction"])
                if json_data["subaction"]
                else None
            ),
        )

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "status": self.status,
            "parameters": self.parameters,
            "subaction": self.subaction.to_json() if self.subaction else None,
        }
