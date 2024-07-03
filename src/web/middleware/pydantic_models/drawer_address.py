from pydantic import BaseModel
from typing import Any


class DrawerAddress(BaseModel):
    robot_name: str
    module_id: int
    drawer_id: int

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "DrawerAddress":
        return cls(
            robot_name=json_data["robot_name"],
            module_id=json_data["module_id"],
            drawer_id=json_data["drawer_id"],
        )
