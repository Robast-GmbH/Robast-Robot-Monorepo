from pydantic import BaseModel
from typing import Any, Dict


class SubmoduleAddress(BaseModel):
    robot_name: str
    module_id: int
    submodule_id: int

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "SubmoduleAddress":
        return cls(
            robot_name=json_data["robot_name"],
            module_id=json_data["module_id"],
            submodule_id=json_data["submodule_id"],
        )

    def to_json(self) -> Dict[str, Any]:
        return self.model_dump()
