from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class Drawer:
    robot_name: str
    module_id: int
    drawer_id: int
    position: int
    size: int
    variant: str
    module_process_status: str
    module_process_type: str
    module_process_payload: Dict[str, int]
    content: Dict[str, int]
    reserved_for_ids: list[str]
    reserved_for_groups: list[str]

    @classmethod
    def from_json(cls, json_data: dict[str, Any]) -> "Drawer":
        return cls(
            robot_name=json_data["robot_name"],
            module_id=json_data["module_id"],
            drawer_id=json_data["drawer_id"],
            size=json_data["size"],
            position=json_data["position"],
            variant=json_data["variant"],
            module_process_status=json_data["module_process_status"],
            module_process_type=json_data["module_process_type"],
            module_process_payload=json_data["module_process_payload"],
            content=json_data["content"],
            reserved_for_ids=json_data["reserved_for_ids"],
            reserved_for_groups=json_data["reserved_for_groups"],
        )

    def to_json(self) -> dict[str, Any]:
        return self.__dict__
